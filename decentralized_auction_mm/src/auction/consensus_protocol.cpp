#include "decentralized_auction_mm/auction/consensus_protocol.hpp"
#include <cmath>
#include <algorithm>

namespace decentralized_auction_mm {
namespace auction {

ConsensusProtocol::ConsensusProtocol(
    uint32_t robot_id,
    const std::map<std::string, double>& params)
    : robot_id_(robot_id),
      params_(params),
      consensus_error_(0.0)
{
    // Set consensus parameters
    gamma_ = 0.5;  // Default base weight factor
    lambda_ = 0.1; // Default information decay rate
    
    if (params.find("gamma") != params.end()) {
        gamma_ = params.at("gamma");
    }
    
    if (params.find("lambda") != params.end()) {
        lambda_ = params.at("lambda");
    }
}

void ConsensusProtocol::processRobotStatus(const msg::RobotStatus& status) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Store robot status
    robot_status_[status.robot_id] = status;
    
    // Update last update time
    last_update_times_[status.robot_id] = 0; // Just received, so 0 time units old
    
    // Increment all other robots' last update times
    for (auto& [robot_id, last_update_time] : last_update_times_) {
        if (robot_id != status.robot_id) {
            last_update_time++;
        }
    }
}

std::map<uint32_t, uint32_t> ConsensusProtocol::updateTaskAssignments(
    const std::map<uint32_t, uint32_t>& local_assignments) {
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // If no other robots or all have failed, just return local assignments
    bool all_others_failed = true;
    for (const auto& [robot_id, status] : robot_status_) {
        if (robot_id != robot_id_ && !status.failed) {
            all_others_failed = false;
            break;
        }
    }
    
    if (robot_status_.size() <= 1 || all_others_failed) {
        return local_assignments;
    }
    
    // Extract task IDs if not already stored
    if (task_ids_.empty()) {
        task_ids_.reserve(local_assignments.size());
        for (const auto& [task_id, _] : local_assignments) {
            task_ids_.push_back(task_id);
        }
        std::sort(task_ids_.begin(), task_ids_.end());
    }
    
    // Create local state vector (assignments only)
    std::map<uint32_t, double> empty_prices;
    Eigen::VectorXd local_state = createStateVector(local_assignments, empty_prices);
    
    // Initialize consensus state with local state
    Eigen::VectorXd consensus_state = local_state;
    
    // Update state based on other robots' information
    for (const auto& [robot_id, status] : robot_status_) {
        if (robot_id != robot_id_ && !status.failed) {
            // Calculate time-weighted factor
            double weight = calculateTimeWeight(last_update_times_[robot_id]);
            
            // Get other robot's assignments from its status
            std::map<uint32_t, uint32_t> other_assignments;
            for (size_t i = 0; i < status.assigned_tasks.size(); ++i) {
                other_assignments[status.assigned_tasks[i]] = robot_id;
            }
            
            // Create other robot's state vector
            Eigen::VectorXd other_state = createStateVector(other_assignments, empty_prices);
            
            // Update consensus state
            consensus_state += weight * (other_state - local_state);
        }
    }
    
    // Extract updated assignments from consensus state
    std::map<uint32_t, uint32_t> updated_assignments;
    std::map<uint32_t, double> dummy_prices;
    extractStateComponents(consensus_state, task_ids_, updated_assignments, dummy_prices);
    
    // Calculate consensus error (L2 norm of the difference)
    consensus_error_ = (consensus_state - local_state).norm();
    
    return updated_assignments;
}

std::map<uint32_t, double> ConsensusProtocol::updateTaskPrices(
    const std::map<uint32_t, double>& local_prices) {
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // If no other robots or all have failed, just return local prices
    bool all_others_failed = true;
    for (const auto& [robot_id, status] : robot_status_) {
        if (robot_id != robot_id_ && !status.failed) {
            all_others_failed = false;
            break;
        }
    }
    
    if (robot_status_.size() <= 1 || all_others_failed) {
        return local_prices;
    }
    
    // Extract task IDs if not already stored
    if (task_ids_.empty()) {
        task_ids_.reserve(local_prices.size());
        for (const auto& [task_id, _] : local_prices) {
            task_ids_.push_back(task_id);
        }
        std::sort(task_ids_.begin(), task_ids_.end());
    }
    
    // Create local state vector (prices only)
    std::map<uint32_t, uint32_t> empty_assignments;
    Eigen::VectorXd local_state = createStateVector(empty_assignments, local_prices);
    
    // Initialize consensus state with local state
    Eigen::VectorXd consensus_state = local_state;
    
    // Update state based on other robots' information
    // Note: In a real implementation, we'd need a way to obtain other robots' price estimates
    // For this simplified version, we rely on broadcasting and receiving price updates
    for (const auto& [robot_id, state] : robot_states_) {
        if (robot_id != robot_id_ && !robot_status_[robot_id].failed) {
            // Calculate time-weighted factor
            double weight = calculateTimeWeight(last_update_times_[robot_id]);
            
            // Update consensus state
            consensus_state += weight * (state - local_state);
        }
    }
    
    // Extract updated prices from consensus state
    std::map<uint32_t, double> updated_prices;
    std::map<uint32_t, uint32_t> dummy_assignments;
    extractStateComponents(consensus_state, task_ids_, dummy_assignments, updated_prices);
    
    // Calculate consensus error (L2 norm of the difference)
    consensus_error_ = (consensus_state - local_state).norm();
    
    return updated_prices;
}

double ConsensusProtocol::calculateTimeWeight(uint64_t last_update_time) {
    return gamma_ * std::exp(-lambda_ * last_update_time);
}

Eigen::VectorXd ConsensusProtocol::createStateVector(
    const std::map<uint32_t, uint32_t>& assignments,
    const std::map<uint32_t, double>& prices) {
    
    // Create a state vector with structure:
    // [assignment_1, assignment_2, ..., price_1, price_2, ...]
    Eigen::VectorXd state = Eigen::VectorXd::Zero(2 * task_ids_.size());
    
    for (size_t i = 0; i < task_ids_.size(); ++i) {
        uint32_t task_id = task_ids_[i];
        
        // Add assignment
        if (assignments.find(task_id) != assignments.end()) {
            state(i) = assignments.at(task_id);
        }
        
        // Add price
        if (prices.find(task_id) != prices.end()) {
            state(i + task_ids_.size()) = prices.at(task_id);
        }
    }
    
    return state;
}

void ConsensusProtocol::extractStateComponents(
    const Eigen::VectorXd& state,
    const std::vector<uint32_t>& task_ids,
    std::map<uint32_t, uint32_t>& assignments,
    std::map<uint32_t, double>& prices) {
    
    for (size_t i = 0; i < task_ids.size(); ++i) {
        uint32_t task_id = task_ids[i];
        
        // Extract assignment (round to nearest integer)
        int assignment = std::round(state(i));
        if (assignment > 0) {
            assignments[task_id] = assignment;
        }
        
        // Extract price
        prices[task_id] = state(i + task_ids.size());
    }
}

double ConsensusProtocol::getConvergenceRate() const {
    // Theoretical convergence rate from section 5.3 of mathematical foundations
    return -std::log(1.0 - 2.0 * gamma_);
}

double ConsensusProtocol::getConsensusError() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return consensus_error_;
}

} // namespace auction
} // namespace decentralized_auction_mm