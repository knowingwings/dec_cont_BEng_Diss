#ifndef DECENTRALIZED_AUCTION_MM_RECOVERY_MECHANISM_HPP
#define DECENTRALIZED_AUCTION_MM_RECOVERY_MECHANISM_HPP

#include <vector>
#include <map>
#include <memory>
#include <mutex>
#include "decentralized_auction_mm/msg/task.hpp"

namespace decentralized_auction_mm {
namespace auction {

/**
 * @class RecoveryMechanism
 * @brief Implements failure recovery for distributed auction algorithm
 * 
 * This class implements the recovery mechanism described in the dissertation,
 * used to reassign tasks after robot failures.
 */
class RecoveryMechanism {
public:
    /**
     * @brief Constructor
     * @param robot_id The ID of the robot
     * @param params The recovery parameters
     */
    RecoveryMechanism(
        uint32_t robot_id,
        const std::map<std::string, double>& params
    );

    /**
     * @brief Initialize recovery after robot failure
     * @param failed_robot_id The ID of the failed robot
     * @param failed_tasks Tasks that were assigned to the failed robot
     * @param criticality_scores Criticality scores for the failed tasks
     */
    void initialize(
        uint32_t failed_robot_id,
        const std::vector<uint32_t>& failed_tasks,
        const std::map<uint32_t, double>& criticality_scores
    );

    /**
     * @brief Prioritize recovery tasks based on criticality
     * @param tasks All tasks
     * @return Prioritized list of task IDs
     */
    std::vector<uint32_t> prioritizeTasks(const std::vector<msg::Task>& tasks);

    /**
     * @brief Calculate recovery bid value for a task
     * @param task The task to bid on
     * @param current_progress Progress on the task (0.0 to 1.0)
     * @param base_bid The base bid value
     * @return Modified bid value for recovery
     */
    double calculateRecoveryBid(
        const msg::Task& task,
        double current_progress,
        double base_bid
    );

    /**
     * @brief Get recovery theoretical time bound
     * @param num_failed_tasks Number of tasks to reassign
     * @param b_max Maximum bid value
     * @param epsilon Minimum bid increment
     * @return Theoretical bound on recovery time
     */
    uint32_t getRecoveryTimeBound(
        uint32_t num_failed_tasks,
        double b_max,
        double epsilon
    ) const;

    /**
     * @brief Get makespan degradation bound
     * @param failed_tasks Tasks assigned to failed robot
     * @param task_progress Current progress on tasks
     * @return Theoretical bound on makespan degradation
     */
    double getMakespanDegradationBound(
        const std::vector<msg::Task>& failed_tasks,
        const std::map<uint32_t, double>& task_progress
    ) const;

private:
    /**
     * @brief Calculate task criticality
     * @param task_id Task ID
     * @param tasks All tasks
     * @return Criticality score
     */
    double calculateTaskCriticality(
        uint32_t task_id,
        const std::vector<msg::Task>& tasks
    );

    /**
     * @brief Calculate task urgency
     * @param task The task
     * @return Urgency score
     */
    double calculateTaskUrgency(const msg::Task& task);

    // Robot ID
    uint32_t robot_id_;

    // Recovery parameters
    std::map<std::string, double> params_;
    
    // Recovery state
    uint32_t failed_robot_id_;
    std::vector<uint32_t> failed_tasks_;
    std::map<uint32_t, double> criticality_scores_;
    std::map<uint32_t, double> initial_progress_;
    
    // Recovery metrics
    uint32_t recovery_start_time_;
    uint32_t recovery_iterations_;

    // Thread safety
    mutable std::mutex mutex_;
};

} // namespace auction
} // namespace decentralized_auction_mm

#endif // DECENTRALIZED_AUCTION_MM_RECOVERY_MECHANISM_HPP