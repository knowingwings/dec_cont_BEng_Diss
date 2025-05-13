#include "decentralized_auction_mm/auction/recovery_mechanism.hpp"
#include <algorithm>
#include <cmath>

namespace decentralized_auction_mm {
namespace auction {

RecoveryMechanism::RecoveryMechanism(
    uint32_t robot_id,
    const std::map<std::string, double>& params)
    : robot_id_(robot_id),
      params_(params),
      failed_robot_id_(0),
      recovery_start_time_(0),
      recovery_iterations_(0)
{
    // Initialize with empty values
}

void RecoveryMechanism::initialize(
    uint32_t failed_robot_id,
    const std::vector<uint32_t>& failed_tasks,
    const std::map<uint32_t, double>& criticality_scores)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    failed_robot_id_ = failed_robot_id;
    failed_tasks_ = failed_tasks;
    criticality_scores_ = criticality_scores;
    
    // Reset recovery metrics
    recovery_start_time_ = static_cast<uint32_t>(time(nullptr));
    recovery_iterations_ = 0;
    
    // Initialize progress as zero for all failed tasks
    for (uint32_t task_id : failed_tasks) {
        initial_progress_[task_id] = 0.0;
    }
}

std::vector<uint32_t> RecoveryMechanism::prioritizeTasks(const std::vector<msg::Task>& tasks)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Get failed tasks
    std::vector<uint32_t> prioritized_tasks;
    for (const auto& task : tasks) {
        // Only consider tasks that were assigned to the failed robot
        if (std::find(failed_tasks_.begin(), failed_tasks_.end(), task.id) != failed_tasks_.end()) {
            prioritized_tasks.push_back(task.id);
        }
    }
    
    // Sort tasks by criticality (highest first)
    std::sort(prioritized_tasks.begin(), prioritized_tasks.end(),
        [this](uint32_t a, uint32_t b) {
            return criticality_scores_[a] > criticality_scores_[b];
        });
    
    return prioritized_tasks;
}

double RecoveryMechanism::calculateRecoveryBid(
    const msg::Task& task,
    double current_progress,
    double base_bid)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Get recovery parameters
    std::vector<double> beta = {2.0, 1.5, 1.0}; // Default values for [progress, criticality, urgency]
    
    if (params_.find("beta1") != params_.end()) beta[0] = params_["beta1"];
    if (params_.find("beta2") != params_.end()) beta[1] = params_["beta2"];
    if (params_.find("beta3") != params_.end()) beta[2] = params_["beta3"];
    
    // 1. Progress term: higher bid for less progress (need to complete more)
    double progress_term = beta[0] * (1.0 - current_progress);
    
    // 2. Criticality term: higher bid for more critical tasks
    double criticality = 0.5; // Default value if not in criticality map
    if (criticality_scores_.find(task.id) != criticality_scores_.end()) {
        criticality = criticality_scores_[task.id];
    }
    double criticality_term = beta[1] * criticality;
    
    // 3. Urgency term: higher bid for more urgent tasks
    double urgency = calculateTaskUrgency(task);
    double urgency_term = beta[2] * urgency;
    
    // Combine terms with base bid
    double recovery_bid = base_bid + progress_term + criticality_term + urgency_term;
    
    // Increment recovery iterations
    recovery_iterations_++;
    
    return recovery_bid;
}

double RecoveryMechanism::calculateTaskUrgency(const msg::Task& task)
{
    // A simple model: tasks with more dependent tasks are more urgent
    double urgency = 0.0;
    
    // This could be enhanced with task deadlines or temporal constraints
    if (task.is_collaborative) {
        // Collaborative tasks are more urgent
        urgency += 0.5;
    }
    
    // Time since recovery started could factor into urgency
    uint32_t current_time = static_cast<uint32_t>(time(nullptr));
    uint32_t elapsed_time = current_time - recovery_start_time_;
    
    // Increase urgency if recovery is taking too long
    if (elapsed_time > 30) { // 30 seconds threshold
        urgency += 0.2 * (elapsed_time - 30) / 30.0; // Linear increase with time
        urgency = std::min(urgency, 1.0); // Cap at 1.0
    }
    
    return urgency;
}

double RecoveryMechanism::calculateTaskCriticality(
    uint32_t task_id,
    const std::vector<msg::Task>& tasks)
{
    // Calculate criticality based on:
    // 1. Number of dependent tasks
    // 2. Execution time
    
    double criticality = 0.0;
    
    // Find the task
    auto task_it = std::find_if(tasks.begin(), tasks.end(),
        [task_id](const msg::Task& task) { return task.id == task_id; });
    
    if (task_it == tasks.end()) {
        return criticality;
    }
    
    // Add execution time component
    criticality += task_it->execution_time;
    
    // Add dependent tasks component
    for (const auto& task : tasks) {
        bool is_dependent = false;
        for (uint32_t prereq : task.prerequisites) {
            if (prereq == task_id) {
                is_dependent = true;
                break;
            }
        }
        
        if (is_dependent) {
            criticality += 2.0; // Each dependent task adds 2.0 to criticality
        }
    }
    
    return criticality;
}

uint32_t RecoveryMechanism::getRecoveryTimeBound(
    uint32_t num_failed_tasks,
    double b_max,
    double epsilon) const
{
    // Theoretical bound: O(|T_f|) + O(b_max/epsilon)
    // From Theorem 6.1 in mathematical foundations
    return num_failed_tasks + static_cast<uint32_t>(std::ceil(b_max / epsilon));
}

double RecoveryMechanism::getMakespanDegradationBound(
    const std::vector<msg::Task>& failed_tasks,
    const std::map<uint32_t, double>& task_progress) const
{
    // Theoretical bound: ΔM ≤ 2 * Σ_{j∈T_f} (1 - p_j(t_f)) * τ(t_j)
    // From Theorem 6.2 in mathematical foundations
    
    double degradation_bound = 0.0;
    
    for (const auto& task : failed_tasks) {
        double progress = 0.0;
        if (task_progress.find(task.id) != task_progress.end()) {
            progress = task_progress.at(task.id);
        }
        
        degradation_bound += 2.0 * (1.0 - progress) * task.execution_time;
    }
    
    return degradation_bound;
}

} // namespace auction
} // namespace decentralized_auction_mms