#include "decentralized_auction_mm/auction/auction_algorithm.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace decentralized_auction_mm {
namespace auction {

AuctionAlgorithm::AuctionAlgorithm(
    uint32_t robot_id,
    rclcpp::Node* node,
    const std::map<std::string, double>& params)
    : robot_id_(robot_id),
      node_(node),
      params_(params),
      iteration_(0),
      in_recovery_mode_(false),
      recovery_iteration_(0),
      failed_robot_id_(0),
      logger_(node->get_logger())
{
    // Initialize component modules
    bid_calculator_ = std::make_unique<BidCalculator>(robot_id, params);
    consensus_protocol_ = std::make_unique<ConsensusProtocol>(robot_id, params);
    recovery_mechanism_ = std::make_unique<RecoveryMechanism>(robot_id, params);

    RCLCPP_INFO(logger_, "Auction algorithm initialized for robot %d", robot_id);
}

void AuctionAlgorithm::initialize(const std::vector<msg::Task>& tasks) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    tasks_ = tasks;
    iteration_ = 0;
    in_recovery_mode_ = false;
    recovery_iteration_ = 0;
    
    // Initialize task-related maps
    for (const auto& task : tasks) {
        task_prices_[task.id] = 0.0;
        task_assignments_[task.id] = 0; // Unassigned
        initial_assignments_[task.id] = 0;
        task_progress_[task.id] = 0.0;
        task_oscillation_count_[task.id] = 0;
        task_last_robot_[task.id] = 0;
        unassigned_iterations_[task.id] = 0;
    }
    
    RCLCPP_INFO(logger_, "Auction initialized with %zu tasks", tasks.size());
}

bool AuctionAlgorithm::processAuctionStep(const std::vector<uint32_t>& available_tasks) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Increment iteration counter
    iteration_++;
    
    if (in_recovery_mode_) {
        recovery_iteration_++;
        return processRecoveryStep(available_tasks);
    }
    
    // Track unassigned iterations counter for all tasks
    for (const auto& task : tasks_) {
        if (task_assignments_[task.id] == 0) {
            unassigned_iterations_[task.id]++;
        } else {
            unassigned_iterations_[task.id] = 0;
        }
    }
    
    // Filter the available tasks list to only include unassigned tasks or tasks
    // that are already assigned to this robot (can be reassigned for better balance)
    std::vector<uint32_t> valid_tasks;
    for (uint32_t task_id : available_tasks) {
        if (task_assignments_[task_id] == 0 || 
            task_assignments_[task_id] == robot_id_) {
            valid_tasks.push_back(task_id);
        }
    }
    
    // Skip if no valid tasks
    if (valid_tasks.empty()) {
        return false;
    }
    
    // Calculate bids for available tasks
    std::vector<msg::Bid> bids = calculateBids(valid_tasks);
    
    // Update assignments based on bids
    bool new_assignments = updateAssignments(bids);
    
    // Adjust prices for unassigned tasks
    if (iteration_ > 10) {
        for (const auto& task : tasks_) {
            if (task_assignments_[task.id] == 0) {
                uint32_t unassigned_iter = unassigned_iterations_[task.id];
                
                // Progressive price reduction for longer unassigned tasks
                double reduction_factor = 0.9; // Default mild reduction
                
                if (unassigned_iter > 30) {
                    reduction_factor = 0.3; // Very aggressive reduction
                } else if (unassigned_iter > 20) {
                    reduction_factor = 0.5; // Strong reduction
                } else if (unassigned_iter > 10) {
                    reduction_factor = 0.7; // Moderate reduction
                }
                
                task_prices_[task.id] *= reduction_factor;
                
                // Ensure price doesn't go below zero
                task_prices_[task.id] = std::max(0.0, task_prices_[task.id]);
                
                // Clear oscillation history for very long unassigned tasks
                if (unassigned_iter > 15) {
                    task_oscillation_count_[task.id] = 0;
                }
                
                // Log status for difficult tasks
                if (unassigned_iter > 25 && iteration_ % 5 == 0) {
                    RCLCPP_WARN(logger_, "Task %d remains unassigned for %d iterations - price reduced to %.2f",
                        task.id, unassigned_iter, task_prices_[task.id]);
                }
            }
        }
    }
    
    return new_assignments;
}

std::vector<msg::Bid> AuctionAlgorithm::calculateBids(const std::vector<uint32_t>& available_tasks) {
    std::vector<msg::Bid> bids;
    
    // Calculate total workload for each robot (including our robot)
    std::map<uint32_t, double> robot_workloads;
    for (const auto& [robot_id, status] : robot_status_) {
        if (!robot_failed_[robot_id]) {
            robot_workloads[robot_id] = status.current_workload;
        }
    }
    
    // Add our own workload if not already included
    if (robot_workloads.find(robot_id_) == robot_workloads.end()) {
        robot_workloads[robot_id_] = getCurrentWorkload();
    }
    
    // Find min and max workloads among active robots
    double min_workload = std::numeric_limits<double>::max();
    double max_workload = 0.0;
    for (const auto& [robot_id, workload] : robot_workloads) {
        if (!robot_failed_[robot_id]) {
            min_workload = std::min(min_workload, workload);
            max_workload = std::max(max_workload, workload);
        }
    }
    
    // Calculate workload imbalance
    double workload_diff = max_workload - min_workload;
    double workload_imbalance = (max_workload > 0) ? workload_diff / max_workload : 0.0;
    
    // Calculate workload ratio for this robot
    double workload_ratio = 1.0;
    if (max_workload > 0) {
        workload_ratio = robot_workloads[robot_id_] / max_workload;
    }
    
    // Calculate adaptive epsilon based on iteration
    double base_epsilon = params_["epsilon"];
    if (iteration_ < 10) {
        base_epsilon *= 1.5; // Higher initial epsilon to prevent oscillations
    }
    
    // For each available task, calculate bid
    for (uint32_t task_id : available_tasks) {
        // Find the task in the tasks_ vector
        auto task_it = std::find_if(tasks_.begin(), tasks_.end(),
            [task_id](const msg::Task& task) { return task.id == task_id; });
        
        if (task_it == tasks_.end()) {
            RCLCPP_ERROR(logger_, "Task %d not found in tasks list", task_id);
            continue;
        }
        
        const msg::Task& task = *task_it;
        
        // Calculate bid using bid calculator
        double bid_value = bid_calculator_->calculateBid(
            task, 
            robot_workloads[robot_id_],
            workload_ratio,
            workload_imbalance,
            iteration_,
            unassigned_iterations_[task_id],
            in_recovery_mode_
        );
        
        // Calculate utility (bid minus price)
        double utility = bid_value - task_prices_[task_id];
        
        // Apply a penalty to the utility if this task has recently oscillated between robots
        if (task_oscillation_count_[task_id] > 3 && task_last_robot_[task_id] != robot_id_) {
            utility *= 0.9;
        }
        
        // Special handling for long-unassigned tasks with escalating incentives
        if (task_assignments_[task_id] == 0) {
            uint32_t unassigned_iter = unassigned_iterations_[task_id];
            if (unassigned_iter > 20) {
                double bonus_factor = std::min(3.0, 1.0 + (unassigned_iter - 20) * 0.1);
                utility *= bonus_factor;
            }
        }
        
        // Only include in bids if utility is positive
        if (utility > 0) {
            msg::Bid bid;
            bid.header.stamp = node_->now();
            bid.robot_id = robot_id_;
            bid.task_id = task_id;
            bid.bid_value = bid_value;
            bid.utility = utility;
            bids.push_back(bid);
        }
    }
    
    // Sort bids by utility (descending)
    std::sort(bids.begin(), bids.end(),
        [](const msg::Bid& a, const msg::Bid& b) {
            return a.utility > b.utility;
        });
    
    return bids;
}

bool AuctionAlgorithm::updateAssignments(const std::vector<msg::Bid>& bids) {
    bool new_assignments = false;
    
    // Determine batch size based on multiple factors
    uint32_t base_batch_size;
    if (iteration_ < 10) {
        base_batch_size = std::ceil(bids.size() / 3.0); // More aggressive initially
    } else {
        base_batch_size = std::ceil(bids.size() / 5.0); // Base batch size
    }
    
    // Adjust batch size based on workload imbalance
    double workload_ratio = 1.0;
    double max_workload = 0.0;
    for (const auto& [robot_id, status] : robot_status_) {
        if (!robot_failed_[robot_id]) {
            max_workload = std::max(max_workload, status.current_workload);
        }
    }
    if (max_workload > 0) {
        workload_ratio = getCurrentWorkload() / max_workload;
    }
    
    double workload_imbalance = std::abs(workload_ratio - 1.0);
    double imbalance_factor = 1.0;
    if (workload_imbalance > 0.3) {
        imbalance_factor = 1.5; // Larger batches when imbalance is high
    } else if (workload_imbalance > 0.15) {
        imbalance_factor = 1.25;
    }
    
    // Adjust batch size based on unassigned tasks
    uint32_t unassigned_count = 0;
    for (const auto& [task_id, robot_id] : task_assignments_) {
        if (robot_id == 0) {
            unassigned_count++;
        }
    }
    
    double unassigned_factor = 1.0;
    if (unassigned_count > 0) {
        unassigned_factor = 1.0 + (unassigned_count / static_cast<double>(tasks_.size())) * 0.5;
    }
    
    // Calculate final batch size
    uint32_t max_bids = std::max(2u, static_cast<uint32_t>(std::ceil(
        base_batch_size * imbalance_factor * unassigned_factor)));
    
    // Limit to available bids
    max_bids = std::min(max_bids, static_cast<uint32_t>(bids.size()));
    
    // Process top bids up to max_bids
    uint32_t bid_count = 0;
    for (const auto& bid : bids) {
        if (bid_count >= max_bids) {
            break;
        }
        
        // Check if this is a new assignment or improvement
        if (task_assignments_[bid.task_id] != robot_id_) {
            uint32_t old_assignment = task_assignments_[bid.task_id];
            
            // Track oscillation (robot changes) for this task
            if (old_assignment > 0 && old_assignment != robot_id_) {
                task_oscillation_count_[bid.task_id]++;
            }
            task_last_robot_[bid.task_id] = robot_id_;
            
            // Update assignment
            task_assignments_[bid.task_id] = robot_id_;
            
            // If this is the first assignment, record it for recovery analysis
            if (initial_assignments_[bid.task_id] == 0) {
                initial_assignments_[bid.task_id] = robot_id_;
            }
            
            // Dynamic price increment with multiple factors
            double effective_epsilon = iteration_ < 10 ? 
                params_["epsilon"] * 1.5 : params_["epsilon"];
            
            // Factor 1: Task oscillation history
            if (task_oscillation_count_[bid.task_id] > 2) {
                effective_epsilon *= (1.0 + 0.15 * task_oscillation_count_[bid.task_id]);
            }
            
            // Factor 2: Workload balancing
            if (workload_ratio > 1.2) { // Robot has >20% more workload than average
                effective_epsilon *= 1.5; // Increase price faster
            } else if (workload_ratio < 0.8) { // Robot has <80% of average workload
                effective_epsilon *= 0.7; // Increase price slower
            }
            
            // Cap prices to prevent them from getting too high
            double max_price = 3.0 * *std::max_element(params_["alpha"].begin(), params_["alpha"].end());
            if (task_prices_[bid.task_id] + effective_epsilon > max_price) {
                effective_epsilon = std::max(0.0, max_price - task_prices_[bid.task_id]);
            }
            
            // Update task price
            updateTaskPrice(bid.task_id, bid.bid_value + effective_epsilon);
            
            new_assignments = true;
            bid_count++;
            
            RCLCPP_DEBUG(logger_, "Robot %d assigned task %d (bid: %.2f, price: %.2f)",
                robot_id_, bid.task_id, bid.bid_value, task_prices_[bid.task_id]);
        }
    }
    
    return new_assignments;
}

void AuctionAlgorithm::updateTaskPrice(uint32_t task_id, double bid_value) {
    task_prices_[task_id] = bid_value;
    
    // Update the task price in the tasks_ vector as well
    for (auto& task : tasks_) {
        if (task.id == task_id) {
            task.current_price = bid_value;
            break;
        }
    }
}

void AuctionAlgorithm::processBid(const msg::Bid& bid) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Ignore bids from failed robots
    if (robot_failed_[bid.robot_id]) {
        return;
    }
    
    // Ignore outdated bids (from previous iterations)
    // This would require adding an iteration field to the Bid message
    
    // Check if the bid is higher than the current price
    if (bid.bid_value > task_prices_[bid.task_id]) {
        uint32_t old_assignment = task_assignments_[bid.task_id];
        
        // Update task assignment and price
        task_assignments_[bid.task_id] = bid.robot_id;
        updateTaskPrice(bid.task_id, bid.bid_value + params_["epsilon"]);
        
        // Update task in tasks_ vector
        for (auto& task : tasks_) {
            if (task.id == bid.task_id) {
                task.assigned_robot = bid.robot_id;
                break;
            }
        }
        
        // Track oscillation
        if (old_assignment > 0 && old_assignment != bid.robot_id) {
            task_oscillation_count_[bid.task_id]++;
        }
        task_last_robot_[bid.task_id] = bid.robot_id;
        
        // If this is the first assignment, record it for recovery analysis
        if (initial_assignments_[bid.task_id] == 0) {
            initial_assignments_[bid.task_id] = bid.robot_id;
        }
        
        RCLCPP_DEBUG(logger_, "Processed bid from robot %d for task %d (bid: %.2f, price: %.2f)",
            bid.robot_id, bid.task_id, bid.bid_value, task_prices_[bid.task_id]);
    }
}

void AuctionAlgorithm::processRobotStatus(const msg::RobotStatus& status) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    robot_status_[status.robot_id] = status;
    robot_workloads_[status.robot_id] = status.current_workload;
    robot_failed_[status.robot_id] = status.failed;
    
    // Check for robot failure (first time detection)
    if (status.failed && !in_recovery_mode_ && status.robot_id != robot_id_) {
        RCLCPP_WARN(logger_, "Detected failure of robot %d, initiating recovery", status.robot_id);
        initiateRecovery(status.robot_id);
    }
    
    // Update consensus information
    consensus_protocol_->processRobotStatus(status);
}

void AuctionAlgorithm::initiateRecovery(uint32_t failed_robot_id) {
    in_recovery_mode_ = true;
    recovery_iteration_ = 0;
    failed_robot_id_ = failed_robot_id;
    
    // Store current assignment state for recovery analysis
    std::map<uint32_t, uint32_t> failure_assignment = task_assignments_;
    
    // Find tasks assigned to the failed robot
    std::vector<uint32_t> failed_tasks;
    for (const auto& [task_id, robot_id] : task_assignments_) {
        if (robot_id == failed_robot_id) {
            failed_tasks.push_back(task_id);
        }
    }
    
    // Calculate criticality scores for failed tasks
    std::map<uint32_t, double> criticality_scores;
    for (uint32_t task_id : failed_tasks) {
        // Find the task
        auto task_it = std::find_if(tasks_.begin(), tasks_.end(),
            [task_id](const msg::Task& task) { return task.id == task_id; });
        
        if (task_it == tasks_.end()) {
            continue;
        }
        
        // 1. Consider execution time
        criticality_scores[task_id] = task_it->execution_time;
        
        // 2. Add bonus for tasks with dependencies
        for (const auto& task : tasks_) {
            for (uint32_t prereq : task.prerequisites) {
                if (prereq == task_id) {
                    criticality_scores[task_id] += 2.0;
                    break;
                }
            }
        }
    }
    
    // Sort failed tasks by criticality (highest first)
    std::sort(failed_tasks.begin(), failed_tasks.end(),
        [&criticality_scores](uint32_t a, uint32_t b) {
            return criticality_scores[a] > criticality_scores[b];
        });
    
    // Process in order of criticality
    for (size_t i = 0; i < failed_tasks.size(); i++) {
        uint32_t task_id = failed_tasks[i];
        
        // Reset oscillation counter
        task_oscillation_count_[task_id] = 0;
        
        // Mark as in recovery with priority level reflected in the price reset
        task_assignments_[task_id] = 0; // Unassigned
        
        // More aggressive price reset for high-priority tasks
        double priority_factor = 1.0 - static_cast<double>(i) / failed_tasks.size();
        double reset_factor = 0.3 * (1.0 + priority_factor);
        task_prices_[task_id] *= reset_factor;
        
        // Update task in tasks_ vector
        for (auto& task : tasks_) {
            if (task.id == task_id) {
                task.assigned_robot = 0;
                task.current_price = task_prices_[task_id];
                break;
            }
        }
    }
    
    RCLCPP_INFO(logger_, "Recovery initiated for robot %d. %zu tasks need reassignment.",
        failed_robot_id, failed_tasks.size());
    
    // Initialize recovery mechanism
    recovery_mechanism_->initialize(failed_robot_id, failed_tasks, criticality_scores);
}

bool AuctionAlgorithm::processRecoveryStep(const std::vector<uint32_t>& available_tasks) {
    // Check if all tasks from failed robot have been reassigned
    bool all_reassigned = true;
    for (const auto& [task_id, robot_id] : task_assignments_) {
        // Check if task was assigned to the failed robot at time of failure
        if (initial_assignments_[task_id] == failed_robot_id_) {
            // If still unassigned or assigned to failed robot, not all reassigned
            if (robot_id == 0 || robot_id == failed_robot_id_) {
                all_reassigned = false;
                break;
            }
        }
    }
    
    if (all_reassigned) {
        RCLCPP_INFO(logger_, "Recovery completed after %d iterations", recovery_iteration_);
        in_recovery_mode_ = false;
        recovery_iteration_ = 0;
        return false;
    }
    
    // Use a modified bid calculation for recovery mode
    std::vector<uint32_t> recovery_tasks;
    for (uint32_t task_id : available_tasks) {
        // Only consider tasks that were assigned to the failed robot
        // or unassigned tasks that are available
        if (initial_assignments_[task_id] == failed_robot_id_ || 
            task_assignments_[task_id] == 0) {
            recovery_tasks.push_back(task_id);
        }
    }
    
    if (recovery_tasks.empty()) {
        return false;
    }
    
    // Calculate bids with recovery-specific logic
    std::vector<msg::Bid> recovery_bids = calculateBids(recovery_tasks);
    
    // Update assignments based on bids
    bool new_assignments = updateAssignments(recovery_bids);
    
    // More aggressive price reduction during recovery for unassigned tasks
    for (uint32_t task_id : recovery_tasks) {
        if (task_assignments_[task_id] == 0) {
            uint32_t unassigned_iter = unassigned_iterations_[task_id];
            
            // Even more aggressive reduction during recovery
            double reduction_factor = 0.9; // Default
            if (unassigned_iter > 5) {
                reduction_factor = 0.7;
            }
            if (unassigned_iter > 10) {
                reduction_factor = 0.5;
            }
            if (unassigned_iter > 15) {
                reduction_factor = 0.3;
            }
            
            task_prices_[task_id] *= reduction_factor;
            task_prices_[task_id] = std::max(0.0, task_prices_[task_id]);
            
            // Update in tasks vector
            for (auto& task : tasks_) {
                if (task.id == task_id) {
                    task.current_price = task_prices_[task_id];
                    break;
                }
            }
        }
    }
    
    return new_assignments;
}

void AuctionAlgorithm::setRobotFailed(uint32_t robot_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (robot_id == robot_id_) {
        RCLCPP_ERROR(logger_, "Cannot set self as failed");
        return;
    }
    
    robot_failed_[robot_id] = true;
    
    // If not already in recovery mode, initiate recovery
    if (!in_recovery_mode_) {
        initiateRecovery(robot_id);
    }
}

bool AuctionAlgorithm::isInRecoveryMode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return in_recovery_mode_;
}

double AuctionAlgorithm::getCurrentWorkload() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    double workload = 0.0;
    for (const auto& task : tasks_) {
        if (task_assignments_.find(task.id) != task_assignments_.end() &&
            task_assignments_.at(task.id) == robot_id_) {
            workload += task.execution_time;
        }
    }
    
    return workload;
}

std::map<uint32_t, uint32_t> AuctionAlgorithm::getCurrentAssignments() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return task_assignments_;
}

msg::AuctionStatus AuctionAlgorithm::getAuctionStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    msg::AuctionStatus status;
    status.header.stamp = node_->now();
    
    if (in_recovery_mode_) {
        status.status = msg::AuctionStatus::STATUS_RECOVERY;
    } else {
        status.status = msg::AuctionStatus::STATUS_RUNNING;
    }
    
    status.iteration = iteration_;
    status.total_tasks = tasks_.size();
    
    // Count assigned and completed tasks
    status.assigned_tasks = 0;
    status.completed_tasks = 0;
    for (const auto& [task_id, robot_id] : task_assignments_) {
        if (robot_id > 0) {
            status.assigned_tasks++;
        }
        
        // Find task to check completion status
        auto task_it = std::find_if(tasks_.begin(), tasks_.end(),
            [task_id](const msg::Task& task) { return task.id == task_id; });
        
        if (task_it != tasks_.end() && task_it->status == msg::Task::STATUS_COMPLETED) {
            status.completed_tasks++;
        }
    }
    
    status.recovery_iteration = recovery_iteration_;
    status.failed_robot_id = failed_robot_id_;
    
    // Calculate global makespan
    std::map<uint32_t, double> robot_makespans;
    for (const auto& [robot_id, robot_status] : robot_status_) {
        if (!robot_failed_[robot_id]) {
            robot_makespans[robot_id] = 0.0;
        }
    }
    
    // Add our own robot if not in the status map
    if (robot_makespans.find(robot_id_) == robot_makespans.end()) {
        robot_makespans[robot_id_] = 0.0;
    }
    
    // Calculate makespan for each robot
    for (const auto& task : tasks_) {
        uint32_t robot_id = task_assignments_[task.id];
        if (robot_id > 0 && !robot_failed_[robot_id]) {
            robot_makespans[robot_id] += task.execution_time;
        }
    }
    
    // Makespan is the maximum completion time among robots
    status.global_makespan = 0.0;
    for (const auto& [robot_id, makespan] : robot_makespans) {
        status.global_makespan = std::max(status.global_makespan, makespan);
    }
    
    return status;
}

std::vector<msg::Task> AuctionAlgorithm::getCurrentTasks() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Create a copy of the tasks with updated prices and assignments
    std::vector<msg::Task> current_tasks = tasks_;
    
    for (auto& task : current_tasks) {
        task.current_price = task_prices_.at(task.id);
        task.assigned_robot = task_assignments_.at(task.id);
    }
    
    return current_tasks;
}

} // namespace auction
} // namespace decentralized_auction_mm