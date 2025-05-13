#ifndef DECENTRALIZED_AUCTION_MM_TASK_MANAGER_HPP
#define DECENTRALIZED_AUCTION_MM_TASK_MANAGER_HPP

#include <vector>
#include <map>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "decentralized_auction_mm/msg/task.hpp"
#include "decentralized_auction_mm/task/dependency_manager.hpp"

namespace decentralized_auction_mm {
namespace task {

/**
 * @class TaskManager
 * @brief Manages tasks and their dependencies
 * 
 * This class maintains the task list, tracks task status,
 * and determines which tasks are available for assignment.
 */
class TaskManager {
public:
    /**
     * @brief Constructor
     * @param robot_id The ID of the robot
     * @param node The ROS node handle
     */
    TaskManager(
        uint32_t robot_id,
        rclcpp::Node* node
    );

    /**
     * @brief Update the task list
     * @param tasks The new task list
     */
    void updateTasks(const std::vector<msg::Task>& tasks);

    /**
     * @brief Get the list of available tasks (no unmet prerequisites)
     * @return Vector of available task IDs
     */
    std::vector<uint32_t> getAvailableTasks() const;

    /**
     * @brief Get all tasks
     * @return Vector of all tasks
     */
    std::vector<msg::Task> getTasks() const;

    /**
     * @brief Update task status
     * @param task_id The ID of the task to update
     * @param status The new status
     * @param progress The new progress (0.0 to 1.0)
     * @return True if the status was updated
     */
    bool updateTaskStatus(uint32_t task_id, uint8_t status, double progress);

    /**
     * @brief Get task by ID
     * @param task_id The ID of the task
     * @return The task, or empty task if not found
     */
    msg::Task getTask(uint32_t task_id) const;

    /**
     * @brief Check if a task can be executed
     * @param task_id The ID of the task
     * @return True if the task is available
     */
    bool isTaskAvailable(uint32_t task_id) const;

    /**
     * @brief Calculate task criticality
     * @param task_id The ID of the task
     * @return Criticality score
     */
    double calculateTaskCriticality(uint32_t task_id) const;

private:
    /**
     * @brief Update the dependency graph
     */
    void updateDependencyGraph();

    /**
     * @brief Find tasks with no prerequisites
     * @return Vector of task IDs
     */
    std::vector<uint32_t> findRootTasks() const;

    /**
     * @brief Find critical path through dependency graph
     * @return Vector of task IDs on critical path
     */
    std::vector<uint32_t> findCriticalPath() const;

    // Robot ID
    uint32_t robot_id_;

    // ROS node handle
    rclcpp::Node* node_;

    // Task list
    std::vector<msg::Task> tasks_;

    // Task dependency manager
    std::unique_ptr<DependencyManager> dependency_manager_;

    // Cache of available tasks
    mutable std::vector<uint32_t> available_tasks_cache_;
    mutable bool cache_valid_;

    // Thread safety
    mutable std::mutex mutex_;
    
    // Logging
    rclcpp::Logger logger_;
};

} // namespace task
} // namespace decentralized_auction_mm

#endif // DECENTRALIZED_AUCTION_MM_TASK_MANAGER_HPP