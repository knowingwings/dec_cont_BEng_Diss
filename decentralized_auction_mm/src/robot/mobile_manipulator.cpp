#include "decentralized_auction_mm/robot/mobile_manipulator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace decentralized_auction_mm {
namespace robot {

MobileManipulator::MobileManipulator(
    uint32_t robot_id,
    rclcpp::Node* node,
    const std::map<std::string, double>& params)
    : robot_id_(robot_id),
      node_(node),
      params_(params),
      initialized_(false),
      manipulator_moving_(false),
      base_moving_(false),
      has_base_target_(false),
      has_joint_target_(false),
      has_ee_target_(false),
      logger_(node->get_logger())
{
    // Get control parameters
    linear_kp_ = 1.0;
    linear_ki_ = 0.0;
    linear_kd_ = 0.1;
    angular_kp_ = 2.0;
    angular_ki_ = 0.0;
    angular_kd_ = 0.2;
    
    if (params.find("linear_kp") != params.end()) {
        linear_kp_ = params.at("linear_kp");
    }
    
    if (params.find("linear_ki") != params.end()) {
        linear_ki_ = params.at("linear_ki");
    }
    
    if (params.find("linear_kd") != params.end()) {
        linear_kd_ = params.at("linear_kd");
    }
    
    if (params.find("angular_kp") != params.end()) {
        angular_kp_ = params.at("angular_kp");
    }
    
    if (params.find("angular_ki") != params.end()) {
        angular_ki_ = params.at("angular_ki");
    }
    
    if (params.find("angular_kd") != params.end()) {
        angular_kd_ = params.at("angular_kd");
    }
    
    // Initialize PID controller variables
    linear_error_sum_ = 0.0;
    angular_error_sum_ = 0.0;
    prev_linear_error_ = 0.0;
    prev_angular_error_ = 0.0;
    last_error_time_ = node_->now();
    
    RCLCPP_INFO(logger_, "Mobile Manipulator initialized for robot %d", robot_id);
}

bool MobileManipulator::initialize() {
    // Create subscribers
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&MobileManipulator::odomCallback, this, std::placeholders::_1));
    
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&MobileManipulator::jointStateCallback, this, std::placeholders::_1));
    
    manipulator_state_sub_ = node_->create_subscription<open_manipulator_msgs::msg::OpenManipulatorState>(
        "open_manipulator_x_controller/state", 10, 
        std::bind(&MobileManipulator::manipulatorStateCallback, this, std::placeholders::_1));
    
    // Create publishers
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Create service clients
    set_joint_position_client_ = node_->create_client<open_manipulator_msgs::srv::SetJointPosition>(
        "open_manipulator_x_controller/set_joint_position");
    
    set_kinematics_pose_client_ = node_->create_client<open_manipulator_msgs::srv::SetKinematicsPose>(
        "open_manipulator_x_controller/set_kinematics_pose");
    
    // Initialize TF2 listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Wait for services to be available
    while (!set_joint_position_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger_, "Interrupted while waiting for service");
            return false;
        }
        RCLCPP_INFO(logger_, "Waiting for set_joint_position service...");
    }
    
    while (!set_kinematics_pose_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(logger_, "Interrupted while waiting for service");
            return false;
        }
        RCLCPP_INFO(logger_, "Waiting for set_kinematics_pose service...");
    }
    
    RCLCPP_INFO(logger_, "Mobile manipulator services connected");
    
    // Mark as initialized
    initialized_ = true;
    
    return true;
}

bool MobileManipulator::moveBase(const geometry_msgs::msg::Pose& target_pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "Mobile manipulator not initialized");
        return false;
    }
    
    target_base_pose_ = target_pose;
    has_base_target_ = true;
    base_moving_ = true;
    
    RCLCPP_INFO(logger_, "Moving base to position [%.2f, %.2f, %.2f]", 
             target_pose.position.x, target_pose.position.y, target_pose.position.z);
    
    return true;
}

bool MobileManipulator::stopBase() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "Mobile manipulator not initialized");
        return false;
    }
    
    // Send zero velocity command
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
    
    cmd_vel_pub_->publish(cmd_vel);
    
    // Reset target
    has_base_target_ = false;
    base_moving_ = false;
    
    RCLCPP_INFO(logger_, "Base stopped");
    
    return true;
}

bool MobileManipulator::setJointPositions(const std::vector<double>& joint_positions) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "Mobile manipulator not initialized");
        return false;
    }
    
    if (joint_positions.size() != 4) {
        RCLCPP_ERROR(logger_, "Expected 4 joint positions for OpenMANIPULATOR-X, got %zu", 
                  joint_positions.size());
        return false;
    }
    
    auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
    
    // Set joint names and positions
    request->joint_position.joint_name = {"joint1", "joint2", "joint3", "joint4"};
    request->joint_position.position = {
        joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3]
    };
    
    // Set path time (time to reach target position)
    request->path_time = 2.0;  // 2 seconds
    
    // Send request asynchronously
    auto result_future = set_joint_position_client_->async_send_request(request);
    
    // Store target positions
    target_joint_positions_ = joint_positions;
    has_joint_target_ = true;
    manipulator_moving_ = true;
    
    RCLCPP_INFO(logger_, "Setting joint positions to [%.2f, %.2f, %.2f, %.2f]", 
             joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3]);
    
    return true;
}

bool MobileManipulator::setEndEffectorPose(const geometry_msgs::msg::Pose& pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "Mobile manipulator not initialized");
        return false;
    }
    
    auto request = std::make_shared<open_manipulator_msgs::srv::SetKinematicsPose::Request>();
    
    // Set end effector name
    request->end_effector_name = "gripper";
    
    // Set kinematics pose
    request->kinematics_pose.pose = pose;
    
    // Set path time (time to reach target position)
    request->path_time = 2.0;  // 2 seconds
    
    // Send request asynchronously
    auto result_future = set_kinematics_pose_client_->async_send_request(request);
    
    // Store target pose
    target_ee_pose_ = pose;
    has_ee_target_ = true;
    manipulator_moving_ = true;
    
    RCLCPP_INFO(logger_, "Setting end effector pose to position [%.2f, %.2f, %.2f]", 
             pose.position.x, pose.position.y, pose.position.z);
    
    return true;
}

bool MobileManipulator::executeTrajectory(const std::vector<geometry_msgs::msg::Pose>& waypoints) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "Mobile manipulator not initialized");
        return false;
    }
    
    if (waypoints.empty()) {
        RCLCPP_ERROR(logger_, "Empty trajectory");
        return false;
    }
    
    // For now, just set the end effector to the final pose
    // In a real implementation, this would execute the trajectory
    return setEndEffectorPose(waypoints.back());
}

bool MobileManipulator::controlGripper(bool open) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "Mobile manipulator not initialized");
        return false;
    }
    
    auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
    
    // Set joint name and position
    request->joint_position.joint_name = {"gripper"};
    request->joint_position.position = {open ? 0.01 : -0.01};  // Open or close
    
    // Set path time (time to reach target position)
    request->path_time = 1.0;  // 1 second
    
    // Send request asynchronously
    auto result_future = set_joint_position_client_->async_send_request(request);
    
    RCLCPP_INFO(logger_, "Setting gripper to %s", open ? "open" : "close");
    
    return true;
}

geometry_msgs::msg::Pose MobileManipulator::getBasePose() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_base_pose_;
}

std::vector<double> MobileManipulator::getJointPositions() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<double> positions;
    
    // Extract joint positions from joint state
    for (size_t i = 0; i < current_joint_state_.name.size(); ++i) {
        // Only consider the manipulator joints (skip base joints)
        if (current_joint_state_.name[i] == "joint1" ||
            current_joint_state_.name[i] == "joint2" ||
            current_joint_state_.name[i] == "joint3" ||
            current_joint_state_.name[i] == "joint4") {
            positions.push_back(current_joint_state_.position[i]);
        }
    }
    
    return positions;
}

geometry_msgs::msg::Pose MobileManipulator::getEndEffectorPose() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    geometry_msgs::msg::Pose pose;
    
    // Try to get end effector pose from TF
    try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            "base_link", "end_effector_link", tf2::TimePointZero);
        
        pose.position.x = transform.transform.translation.x;
        pose.position.y = transform.transform.translation.y;
        pose.position.z = transform.transform.translation.z;
        pose.orientation = transform.transform.rotation;
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(logger_, "Transform error: %s", ex.what());
        
        // Return identity pose
        pose.orientation.w = 1.0;
    }
    
    return pose;
}

bool MobileManipulator::isManipulatorMoving() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return manipulator_moving_;
}

bool MobileManipulator::isBaseMoving() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return base_moving_;
}

void MobileManipulator::update() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        return;
    }
    
    // Update base movement
    if (has_base_target_ && base_moving_) {
        // Calculate velocity command
        geometry_msgs::msg::Twist cmd_vel = calculateVelocityCommand(target_base_pose_);
        
        // Send velocity command
        cmd_vel_pub_->publish(cmd_vel);
        
        // Check if base has reached target
        double distance = std::sqrt(
            std::pow(current_base_pose_.position.x - target_base_pose_.position.x, 2) +
            std::pow(current_base_pose_.position.y - target_base_pose_.position.y, 2));
        
        // Get current yaw angle
        tf2::Quaternion q_current(
            current_base_pose_.orientation.x,
            current_base_pose_.orientation.y,
            current_base_pose_.orientation.z,
            current_base_pose_.orientation.w);
        tf2::Matrix3x3 m_current(q_current);
        double roll_current, pitch_current, yaw_current;
        m_current.getRPY(roll_current, pitch_current, yaw_current);
        
        // Get target yaw angle
        tf2::Quaternion q_target(
            target_base_pose_.orientation.x,
            target_base_pose_.orientation.y,
            target_base_pose_.orientation.z,
            target_base_pose_.orientation.w);
        tf2::Matrix3x3 m_target(q_target);
        double roll_target, pitch_target, yaw_target;
        m_target.getRPY(roll_target, pitch_target, yaw_target);
        
        // Calculate angular difference
        double angle_diff = std::fabs(yaw_current - yaw_target);
        angle_diff = std::min(angle_diff, 2 * M_PI - angle_diff);
        
        // Check if arrived
        if (distance < 0.05 && angle_diff < 0.1) {
            // Target reached
            base_moving_ = false;
            
            // Stop base
            geometry_msgs::msg::Twist stop_cmd;
            cmd_vel_pub_->publish(stop_cmd);
            
            RCLCPP_INFO(logger_, "Base reached target position");
        }
    }
    
    // Update manipulator state
    if (manipulator_state_ == "STOPPED") {
        manipulator_moving_ = false;
    }
}

void MobileManipulator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Update current pose
    current_base_pose_ = msg->pose.pose;
    current_velocity_ = msg->twist.twist;
}

void MobileManipulator::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Update current joint state
    current_joint_state_ = *msg;
}

void MobileManipulator::manipulatorStateCallback(
    const open_manipulator_msgs::msg::OpenManipulatorState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Update manipulator state
    manipulator_state_ = msg->open_manipulator_moving_state;
    
    // Update moving flag
    manipulator_moving_ = (manipulator_state_ == "MOVING");
}

geometry_msgs::msg::Twist MobileManipulator::calculateVelocityCommand(
    const geometry_msgs::msg::Pose& target_pose) {
    geometry_msgs::msg::Twist cmd_vel;
    
    // Get current pose
    geometry_msgs::msg::Pose current_pose = current_base_pose_;
    
    // Calculate linear error (distance to target)
    double dx = target_pose.position.x - current_pose.position.x;
    double dy = target_pose.position.y - current_pose.position.y;
    double linear_error = std::sqrt(dx*dx + dy*dy);
    
    // Get current yaw angle
    tf2::Quaternion q_current(
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w);
    tf2::Matrix3x3 m_current(q_current);
    double roll_current, pitch_current, yaw_current;
    m_current.getRPY(roll_current, pitch_current, yaw_current);
    
    // Calculate target yaw angle (based on position diff)
    double target_yaw = std::atan2(dy, dx);
    
    // Get target orientation from pose
    tf2::Quaternion q_target(
        target_pose.orientation.x,
        target_pose.orientation.y,
        target_pose.orientation.z,
        target_pose.orientation.w);
    tf2::Matrix3x3 m_target(q_target);
    double roll_target, pitch_target, yaw_target;
    m_target.getRPY(roll_target, pitch_target, yaw_target);
    
    // Calculate angular error
    double angular_error = target_yaw - yaw_current;
    
    // Normalize angular error to [-pi, pi]
    while (angular_error > M_PI) angular_error -= 2 * M_PI;
    while (angular_error < -M_PI) angular_error += 2 * M_PI;
    
    // Calculate time difference
    auto current_time = node_->now();
    double dt = (current_time - last_error_time_).seconds();
    if (dt < 0.001) dt = 0.001;  // Avoid division by zero
    
    // Calculate linear control (PID)
    linear_error_sum_ += linear_error * dt;
    double linear_error_der = (linear_error - prev_linear_error_) / dt;
    double linear_control = linear_kp_ * linear_error + 
                           linear_ki_ * linear_error_sum_ + 
                           linear_kd_ * linear_error_der;
    
    // Calculate angular control (PID)
    angular_error_sum_ += angular_error * dt;
    double angular_error_der = (angular_error - prev_angular_error_) / dt;
    double angular_control = angular_kp_ * angular_error + 
                            angular_ki_ * angular_error_sum_ + 
                            angular_kd_ * angular_error_der;
    
    // Update previous errors and time
    prev_linear_error_ = linear_error;
    prev_angular_error_ = angular_error;
    last_error_time_ = current_time;
    
    // Set maximum velocity limits
    double max_linear_vel = 0.26;  // m/s (TurtleBot3 Waffle Pi limit)
    double max_angular_vel = 1.82;  // rad/s (TurtleBot3 Waffle Pi limit)
    
    if (params_.find("max_linear_vel") != params_.end()) {
        max_linear_vel = params_.at("max_linear_vel");
    }
    
    if (params_.find("max_angular_vel") != params_.end()) {
        max_angular_vel = params_.at("max_angular_vel");
    }
    
    // Apply velocity limits
    linear_control = std::max(-max_linear_vel, std::min(max_linear_vel, linear_control));
    angular_control = std::max(-max_angular_vel, std::min(max_angular_vel, angular_control));
    
    // If close to target, reduce velocity
    if (linear_error < 0.2) {
        linear_control *= linear_error / 0.2;
    }
    
    // If angular error is large, prioritize rotation
    if (std::fabs(angular_error) > 0.3) {
        linear_control *= (1.0 - std::min(1.0, std::fabs(angular_error) / M_PI));
    }
    
    // Set command velocity
    cmd_vel.linear.x = linear_control;
    cmd_vel.angular.z = angular_control;
    
    return cmd_vel;
}

std::vector<double> MobileManipulator::quaternionToEuler(const geometry_msgs::msg::Quaternion& quat) const {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    return {roll, pitch, yaw};
}

} // namespace robot
} // namespace decentralized_auction_mm