#ifndef DECENTRALIZED_AUCTION_MM_MOBILE_MANIPULATOR_HPP
#define DECENTRALIZED_AUCTION_MM_MOBILE_MANIPULATOR_HPP

#include <memory>
#include <mutex>
#include <map>
#include <vector>
#include <string>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <open_manipulator_msgs/msg/open_manipulator_state.hpp>
#include <open_manipulator_msgs/srv/set_joint_position.hpp>
#include <open_manipulator_msgs/srv/set_kinematics_pose.hpp>

namespace decentralized_auction_mm {
namespace robot {

/**
 * @class MobileManipulator
 * @brief Controls a mobile manipulator (TurtleBot3 + OpenMANIPULATOR-X)
 * 
 * This class provides interfaces to control both the mobile base and
 * the manipulator arm, handling kinematics and motion.
 */
class MobileManipulator {
public:
    /**
     * @brief Constructor
     * @param robot_id The ID of the robot
     * @param node The ROS node handle
     * @param params The controller parameters
     */
    MobileManipulator(
        uint32_t robot_id,
        rclcpp::Node* node,
        const std::map<std::string, double>& params
    );

    /**
     * @brief Initialize the mobile manipulator
     * @return True if successful
     */
    bool initialize();

    /**
     * @brief Move the base to a target pose
     * @param target_pose The target pose
     * @return True if movement started successfully
     */
    bool moveBase(const geometry_msgs::msg::Pose& target_pose);

    /**
     * @brief Stop the base movement
     * @return True if successful
     */
    bool stopBase();

    /**
     * @brief Set joint positions for the manipulator
     * @param joint_positions Vector of joint positions
     * @return True if command sent successfully
     */
    bool setJointPositions(const std::vector<double>& joint_positions);

    /**
     * @brief Set end effector pose
     * @param pose The target pose
     * @return True if command sent successfully
     */
    bool setEndEffectorPose(const geometry_msgs::msg::Pose& pose);

    /**
     * @brief Execute a trajectory
     * @param waypoints Vector of pose waypoints
     * @return True if execution started successfully
     */
    bool executeTrajectory(const std::vector<geometry_msgs::msg::Pose>& waypoints);

    /**
     * @brief Control the gripper
     * @param open True to open, false to close
     * @return True if command sent successfully
     */
    bool controlGripper(bool open);

    /**
     * @brief Get current base pose
     * @return Current base pose
     */
    geometry_msgs::msg::Pose getBasePose() const;

    /**
     * @brief Get current joint positions
     * @return Vector of joint positions
     */
    std::vector<double> getJointPositions() const;

    /**
     * @brief Get current end effector pose
     * @return End effector pose
     */
    geometry_msgs::msg::Pose getEndEffectorPose() const;

    /**
     * @brief Check if manipulator is moving
     * @return True if moving
     */
    bool isManipulatorMoving() const;

    /**
     * @brief Check if base is moving
     * @return True if moving
     */
    bool isBaseMoving() const;

    /**
     * @brief Update the mobile manipulator
     * This method should be called periodically
     */
    void update();

private:
    /**
     * @brief Callback for odometry messages
     * @param msg The odometry message
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Callback for joint state messages
     * @param msg The joint state message
     */
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    /**
     * @brief Callback for manipulator state messages
     * @param msg The manipulator state message
     */
    void manipulatorStateCallback(
        const open_manipulator_msgs::msg::OpenManipulatorState::SharedPtr msg);

    /**
     * @brief Calculate velocity command for base movement
     * @param target_pose The target pose
     * @return Velocity command
     */
    geometry_msgs::msg::Twist calculateVelocityCommand(
        const geometry_msgs::msg::Pose& target_pose);
    
    /**
     * @brief Convert quaternion to Euler angles
     * @param quat Quaternion
     * @return Vector of [roll, pitch, yaw]
     */
    std::vector<double> quaternionToEuler(const geometry_msgs::msg::Quaternion& quat) const;

    // Robot ID
    uint32_t robot_id_;

    // ROS node handle
    rclcpp::Node* node_;

    // Controller parameters
    std::map<std::string, double> params_;

    // State variables
    bool initialized_;
    bool manipulator_moving_;
    bool base_moving_;
    bool has_base_target_;
    bool has_joint_target_;
    bool has_ee_target_;
    std::string manipulator_state_;

    // Current state
    geometry_msgs::msg::Pose current_base_pose_;
    geometry_msgs::msg::Twist current_velocity_;
    sensor_msgs::msg::JointState current_joint_state_;

    // Target states
    geometry_msgs::msg::Pose target_base_pose_;
    std::vector<double> target_joint_positions_;
    geometry_msgs::msg::Pose target_ee_pose_;

    // PID control variables
    double linear_kp_;
    double linear_ki_;
    double linear_kd_;
    double angular_kp_;
    double angular_ki_;
    double angular_kd_;
    double linear_error_sum_;
    double angular_error_sum_;
    double prev_linear_error_;
    double prev_angular_error_;
    rclcpp::Time last_error_time_;

    // ROS subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<open_manipulator_msgs::msg::OpenManipulatorState>::SharedPtr manipulator_state_sub_;

    // ROS publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // ROS service clients
    rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr set_joint_position_client_;
    rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedPtr set_kinematics_pose_client_;

    // TF2 buffer and listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Thread safety
    mutable std::mutex mutex_;
    
    // Logging
    rclcpp::Logger logger_;
};

} // namespace robot
} // namespace decentralized_auction_mm

#endif // DECENTRALIZED_AUCTION_MM_MOBILE_MANIPULATOR_HPP