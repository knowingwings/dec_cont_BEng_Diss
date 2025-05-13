#include "decentralized_auction_mm/auction/bid_calculator.hpp"
#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>

namespace decentralized_auction_mm {
namespace auction {

BidCalculator::BidCalculator(
    uint32_t robot_id,
    const std::map<std::string, double>& params)
    : robot_id_(robot_id),
      params_(params)
{
    // Initialize with defaults for position and capabilities
    position_.x = 0.0;
    position_.y = 0.0;
    position_.z = 0.0;
    
    // Initialize with balanced capabilities
    capabilities_ = {1.0, 1.0, 1.0, 1.0, 1.0};
}

double BidCalculator::calculateBid(
    const msg::Task& task,
    double current_workload,
    double workload_ratio,
    double workload_imbalance,
    uint32_t iteration,
    uint32_t unassigned_iterations,
    bool in_recovery_mode)
{
    // Calculate distance factor (higher for closer tasks)
    double d_factor = calculateDistanceFactor(task.position.position);
    
    // Calculate configuration cost factor (lower for easier configurations)
    double c_factor = calculateConfigurationFactor(task.position.position);
    
    // Calculate capability matching (higher for better match)
    double capability_match = calculateCapabilityMatch(
        std::vector<double>(task.capabilities_required.begin(), task.capabilities_required.end()));
    
    // Calculate workload factor (higher for higher workload)
    double workload_factor = calculateWorkloadFactor(current_workload, workload_ratio);
    
    // Calculate energy consumption factor
    double energy_factor = calculateEnergyFactor(task.position.position);
    
    // Get bid component weights from parameters
    std::vector<double> alpha = {0.8, 0.3, 1.0, 1.2, 0.2}; // Default values
    
    if (params_.find("alpha1") != params_.end()) alpha[0] = params_["alpha1"];
    if (params_.find("alpha2") != params_.end()) alpha[1] = params_["alpha2"];
    if (params_.find("alpha3") != params_.end()) alpha[2] = params_["alpha3"];
    if (params_.find("alpha4") != params_.end()) alpha[3] = params_["alpha4"];
    if (params_.find("alpha5") != params_.end()) alpha[4] = params_["alpha5"];
    
    // Adjust workload penalty when imbalance is high
    double adjusted_workload_alpha = alpha[3];
    if (workload_ratio > 1.2 || workload_ratio < 0.8) {
        adjusted_workload_alpha *= 1.5; // 50% increase in penalty
    }
    
    // Calculate global balance factor
    double global_balance_factor = workload_imbalance * 0.5;
    
    // Calculate bonus for unassigned tasks
    double task_unassigned_bonus = calculateUnassignedBonus(unassigned_iterations);
    
    // Calculate time urgency factor
    double iteration_factor = calculateTimeUrgencyFactor(iteration, unassigned_iterations);
    
    // Base bid calculation
    double bid = alpha[0] * d_factor + 
                alpha[1] * c_factor + 
                alpha[2] * capability_match - 
                adjusted_workload_alpha * workload_factor - 
                alpha[4] * energy_factor + 
                global_balance_factor + 
                task_unassigned_bonus + 
                iteration_factor;
    
    // Add recovery-specific terms if in recovery mode
    if (in_recovery_mode) {
        std::vector<double> beta = {2.0, 1.5, 1.0}; // Default recovery weights
        
        if (params_.find("beta1") != params_.end()) beta[0] = params_["beta1"];
        if (params_.find("beta2") != params_.end()) beta[1] = params_["beta2"];
        if (params_.find("beta3") != params_.end()) beta[2] = params_["beta3"];
        
        // No partial progress assumed for now
        double progress_term = beta[0] * (1.0 - 0.0);
        
        // Default criticality
        double criticality_term = beta[1] * 0.5;
        
        // Recovery bids favor robots with lower workload more strongly
        double recovery_workload_factor = (1.0 - workload_ratio) * beta[0] * 0.8;
        
        // Add stronger global balance factor during recovery
        double global_recovery_factor = workload_imbalance * beta[2] * 0.7;
        
        bid += progress_term + criticality_term + recovery_workload_factor + global_recovery_factor;
    }
    
    // Add small random noise to break ties (slightly increased to help prevent cycling)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 0.002);
    bid += dis(gen);
    
    return bid;
}

double BidCalculator::calculateDistanceFactor(const geometry_msgs::msg::Point& task_position) {
    // Calculate Euclidean distance
    double dx = position_.x - task_position.x;
    double dy = position_.y - task_position.y;
    double dz = position_.z - task_position.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    // Return inverted distance (higher for closer tasks)
    return 1.0 / (1.0 + distance);
}

double BidCalculator::calculateConfigurationFactor(const geometry_msgs::msg::Point& task_position) {
    // Simplified configuration cost model
    // This could be expanded with actual robot kinematics
    
    // For now, just use a simplified model based on distance and height
    double height_diff = std::abs(position_.z - task_position.z);
    
    // Return inverted cost (higher for easier configurations)
    return 1.0 / (1.0 + height_diff * 2.0);
}

double BidCalculator::calculateCapabilityMatch(const std::vector<double>& task_capabilities) {
    // Normalize vectors
    std::vector<double> robot_cap_norm = normalizeVector(capabilities_);
    std::vector<double> task_cap_norm = normalizeVector(task_capabilities);
    
    // Compute cosine similarity (normalized dot product)
    double dot_product = 0.0;
    for (size_t i = 0; i < std::min(robot_cap_norm.size(), task_cap_norm.size()); ++i) {
        dot_product += robot_cap_norm[i] * task_cap_norm[i];
    }
    
    // Ensure the match is between 0 and 1
    return std::max(0.0, std::min(1.0, dot_product));
}

double BidCalculator::calculateWorkloadFactor(double workload, double workload_ratio) {
    // Progressive workload factor with stronger imbalance penalty
    // This creates a non-linear penalty that increases more rapidly with workload
    // The exponent (1.8) makes the penalty grow faster than linear
    double base_workload_factor = workload / 10.0;
    return base_workload_factor * std::pow(workload_ratio, 1.8);
}

double BidCalculator::calculateEnergyFactor(const geometry_msgs::msg::Point& task_position) {
    // Simplified energy model based on distance
    double dx = position_.x - task_position.x;
    double dy = position_.y - task_position.y;
    double dz = position_.z - task_position.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    return distance * 0.1; // Simple linear model
}

double BidCalculator::calculateUnassignedBonus(uint32_t unassigned_iterations) {
    // Enhanced bonus for unassigned tasks with progressive scaling
    if (unassigned_iterations > 25) {
        return std::min(6.0, 0.5 * std::exp(unassigned_iterations/10.0));
    } else if (unassigned_iterations > 15) {
        return std::min(4.0, 0.4 * std::exp(unassigned_iterations/12.0));
    } else if (unassigned_iterations > 5) {
        return std::min(3.0, 0.3 * unassigned_iterations);
    }
    return 0.0;
}

double BidCalculator::calculateTimeUrgencyFactor(uint32_t iteration, uint32_t unassigned_iterations) {
    // Add scaling factor for tasks that need to be assigned quickly
    if (iteration > 20 && unassigned_iterations > 0) {
        return std::min(2.0, 0.05 * iteration);
    }
    return 0.0;
}

void BidCalculator::setRobotPosition(const geometry_msgs::msg::Point& position) {
    position_ = position;
}

void BidCalculator::setRobotCapabilities(const std::vector<double>& capabilities) {
    capabilities_ = capabilities;
}

std::vector<double> BidCalculator::normalizeVector(const std::vector<double>& vec) {
    double magnitude = 0.0;
    for (double val : vec) {
        magnitude += val * val;
    }
    magnitude = std::sqrt(magnitude);
    
    if (magnitude < 1e-10) {
        return std::vector<double>(vec.size(), 1.0 / std::sqrt(vec.size()));
    }
    
    std::vector<double> normalized_vec(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        normalized_vec[i] = vec[i] / magnitude;
    }
    
    return normalized_vec;
}

} // namespace auction
} // namespace decentralized_auction_mm