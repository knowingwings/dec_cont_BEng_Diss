#ifndef DECENTRALIZED_AUCTION_MM_ROBOT_CONTROLLER_HPP
#define DECENTRALIZED_AUCTION_MM_ROBOT_CONTROLLER_HPP

#include <memory>
#include <mutex>
#include <map>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "decentralized_auction_mm/msg/task.hpp"
#include "decentralized_auction_mm/msg/robot_status.hpp"
#include "decentralized_auction_mm/robot/task_executor.hpp"
#include "decentralized_auction_mm/robot/mobile_manipulator.hpp"

namespace decentralized_auction_mm {
namespace robot {

/**
 * @class RobotController
 * @brief Controls a mobile manipulator for executing assigned tasks
 * 
 * This class interfaces with the TurtleBot3 and OpenMANIPULATOR-X hardware
 * (or simulation) to execute tasks assigned by the auction algorithm.
 */
class RobotController {
public:
    /**
     * @brief Constructor
     * @param robot_id The ID of the robot
     * @param node The ROS node handle
     * @param params The robot parameters
     */
    RobotController(
        uint32_t robot_id,
        rclcpp::Node* node,
        const std::map<std::string, double>& params
    );

    /**
     * @brief Initialize the robot controller
     */
    void initialize();

    /**
     * @brief Assign a task to the robot
     * @param task The task to assign
     * @return True if the task was successfully assigned
     */
    bool assignTask(const msg::Task& task);

    /**
     * @brief Update the robot controller
     * @return True if the robot is currently executing a task
     */
    bool update();

    /**
     * @brief Get the robot status
     * @return Current robot status message
     */
    msg::RobotStatus getRobotStatus() const;

    /**
     * @brief Set the robot as failed
     * @param failed Whether the robot has failed
     */
    void setFailed(bool failed);

    /**
     * @brief Check if the robot has failed
     * @return True if the robot has failed
     */
    bool hasFailed() const;

    /**
     * @brief Get the current robot position
     * @return Current position
     */
    geometry_msgs::msg::Point getPosition() const;

    /**
     * @brief Get the current robot capabilities
     * @return Capability vector
     */
    std::vector<double> getCapabilities() const;

    /**
     * @brief Get the current robot workload
     * @return Total execution time of assigned tasks
     */
    double getWorkload() const;

    /**
     * @brief Get the IDs of assigned tasks
     * @return Vector of assigned task IDs
     */
    std::vector<uint32_t> getAssignedTasks() const;

private:
    /**
     * @brief Callback for robot odometry
     * @param msg The odometry message
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Callback for joint states
     * @param msg The joint state message
     */
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    /**
     * @brief Execute a task
     * @param task The task to execute
     * @return True if the task was successfully executed
     */
    bool executeTask(const msg::Task& task);

    /**
     * @brief Move the robot to a target position
     * @param target The target position
     * @return True if the robot reached the target
     */
    bool moveToPosition(const geometry_msgs::msg::Point& target);

    /**
     * @brief Configure the manipulator for a task
     * @param task The task to configure for
     * @return True if the manipulator was successfully configured
     */
    bool configureManipulator(const msg::Task& task);

    /**
     * @brief Update progress on the current task
     * @return Current progress (0.0 to 1.0)
     */
    double updateTaskProgress();

    // Robot ID
    uint32_t robot_id_;

    // ROS node handle
    rclcpp::Node* node_;

    // Robot parameters
    std::map<std::string, double> params_;

    // Robot state
    bool initialized_;
    bool failed_;
    geometry_msgs::msg::Pose current_pose_;
    sensor_msgs::msg::JointState current_joint_state_;
    std::vector<double> capabilities_;
    uint64_t heartbeat_;

    // Task execution
    std::queue<msg::Task> task_queue_;
    msg::Task current_task_;
    bool executing_task_;
    double task_progress_;

    // Component modules
    std::unique_ptr<TaskExecutor> task_executor_;
    std::unique_ptr<MobileManipulator> mobile_manipulator_;

    // ROS subscribers and publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // TF listener for transformations
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Thread safety
    mutable std::mutex mutex_;
    
    // Logging
    rclcpp::Logger logger_;
};

} // namespace robot
} // namespace decentralized_auction_mm

#endif // DECENTRALIZED_AUCTION_MM_ROBOT_CONTROLLER_HPP