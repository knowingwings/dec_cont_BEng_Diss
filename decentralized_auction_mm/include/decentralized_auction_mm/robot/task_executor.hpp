#ifndef DECENTRALIZED_AUCTION_MM_TASK_EXECUTOR_HPP
#define DECENTRALIZED_AUCTION_MM_TASK_EXECUTOR_HPP

#include <memory>
#include <mutex>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "decentralized_auction_mm/msg/task.hpp"
#include "decentralized_auction_mm/robot/mobile_manipulator.hpp"  // Include the full definition

namespace decentralized_auction_mm {
namespace robot {

/**
 * @class TaskExecutor
 * @brief Executes assembly tasks using a mobile manipulator
 * 
 * This class implements the logic for executing assembly tasks,
 * coordinating the mobile base and manipulator movements.
 */
class TaskExecutor {
public:
    /**
     * @brief Constructor
     * @param robot_id The ID of the robot
     * @param node The ROS node handle
     * @param mobile_manipulator Pointer to the mobile manipulator controller
     * @param params The executor parameters
     */
    TaskExecutor(
        uint32_t robot_id,
        rclcpp::Node* node,
        MobileManipulator* mobile_manipulator,
        const std::map<std::string, double>& params
    );

    /**
     * @brief Start executing a task
     * @param task The task to execute
     * @return True if the task was successfully started
     */
    bool startTask(const msg::Task& task);

    /**
     * @brief Update the task execution
     * @return Progress of the task (0.0 to 1.0), or -1.0 if completed
     */
    double update();

    /**
     * @brief Check if a task is currently being executed
     * @return True if a task is being executed
     */
    bool isExecuting() const;

    /**
     * @brief Get the current task
     * @return The current task, or empty task if none
     */
    msg::Task getCurrentTask() const;

    /**
     * @brief Cancel the current task
     * @return True if a task was cancelled
     */
    bool cancelTask();

    /**
     * @brief Check if this executor can handle a task
     * @param task The task to check
     * @return True if the task can be executed
     */
    bool canExecuteTask(const msg::Task& task) const;

private:
    /**
     * @brief Plan execution sequence for a task
     * @param task The task to plan for
     * @return True if planning was successful
     */
    bool planTaskExecution(const msg::Task& task);

    /**
     * @brief Move base to task position
     * @return True if the movement was completed
     */
    bool moveBaseToTask();

    /**
     * @brief Configure manipulator for task
     * @return True if the configuration was completed
     */
    bool configureManipulator();

    /**
     * @brief Execute manipulation sequence
     * @return True if the manipulation was completed
     */
    bool executeManipulation();

    /**
     * @brief Finish task execution
     * @return True if the task was successfully finished
     */
    bool finishTask();

    /**
     * @brief Handle collaborative synchronization
     * @return True if synchronized successfully
     */
    bool handleSynchronization();

    // Robot ID
    uint32_t robot_id_;

    // ROS node handle
    rclcpp::Node* node_;

    // Mobile manipulator controller
    MobileManipulator* mobile_manipulator_;

    // Executor parameters
    std::map<std::string, double> params_;

    // Task execution state
    enum class ExecutionState {
        IDLE,
        MOVING_TO_TASK,
        CONFIGURING_MANIPULATOR,
        WAITING_FOR_SYNC,
        EXECUTING_MANIPULATION,
        FINISHING
    };

    ExecutionState state_;
    msg::Task current_task_;
    double progress_;
    rclcpp::Time task_start_time_;
    bool task_completed_;

    // Execution plan
    geometry_msgs::msg::Pose target_base_pose_;
    std::vector<double> target_joint_positions_;
    std::vector<geometry_msgs::msg::Pose> manipulation_waypoints_;
    uint32_t current_waypoint_;

    // Synchronization for collaborative tasks
    bool is_leader_;
    bool sync_received_;
    rclcpp::Time sync_request_time_;

    // Thread safety
    mutable std::mutex mutex_;
    
    // Logging
    rclcpp::Logger logger_;
};

} // namespace robot
} // namespace decentralized_auction_mm

#endif // DECENTRALIZED_AUCTION_MM_TASK_EXECUTOR_HPP