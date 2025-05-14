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
      manipulator_state_("STOPPED"),
      linear_kp_(1.0),
      linear_ki_(0.0),
      linear_kd_(0.1),
      angular_kp_(2.0),
      angular_ki_(0.0),
      angular_kd_(0.2),
      linear_error_sum_(0.0),
      angular_error_sum_(0.0),
      prev_linear_error_(0.0),
      prev_angular_error_(0.0),
      logger_(node->get_logger())
{
    // Get control parameters
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
    
    // Initialize current_base_pose_
    current_base_pose_.position.x = 0.0;
    current_base_pose_.position.y = 0.0;
    current_base_pose_.position.z = 0.0;
    current_base_pose_.orientation.x = 0.0;
    current_base_pose_.orientation.y = 0.0;
    current_base_pose_.orientation.z = 0.0;
    current_base_pose_.orientation.w = 1.0;
    
    // Initialize current_velocity_
    current_velocity_.linear.x = 0.0;
    current_velocity_.linear.y = 0.0;
    current_velocity_.linear.z = 0.0;
    current_velocity_.angular.x = 0.0;
    current_velocity_.angular.y = 0.0;
    current_velocity_.angular.z = 0.0;
    
    // Initialize last_error_time_
    last_error_time_ = node_->now();
    
    RCLCPP_INFO(logger_, "Mobile Manipulator initialized for robot %d", robot_id);
}

bool MobileManipulator::initialize() {
    // Create subscribers
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&MobileManipulator::odomCallback, this, std::placeholders::_1));
    
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&MobileManipulator::jointStateCallback, this, std::placeholders::_1));
    
#ifdef HAVE_OPEN_MANIPULATOR
    // Initialize MoveIt interfaces
    try {
        // Allow time for node initialization
        RCLCPP_INFO(logger_, "Waiting for MoveIt to initialize...");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Create MoveGroup instances
        move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, "arm");
        move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, "gripper");
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        
        RCLCPP_INFO(logger_, "MoveIt initialization successful");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "MoveIt initialization failed: %s", e.what());
        return false;
    }
#endif
    
    // Create publishers
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Initialize TF2 listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    RCLCPP_INFO(logger_, "Mobile manipulator initialization complete");
    
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
    
    // Store target pose
    target_base_pose_ = target_pose;
    has_base_target_ = true;
    base_moving_ = true;
    
    // Reset PID controller variables
    linear_error_sum_ = 0.0;
    angular_error_sum_ = 0.0;
    prev_linear_error_ = 0.0;
    prev_angular_error_ = 0.0;
    last_error_time_ = node_->now();
    
    RCLCPP_INFO(logger_, "Moving base to position [%.2f, %.2f]", 
             target_pose.position.x, target_pose.position.y);
    
    return true;
}

bool MobileManipulator::stopBase() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "Mobile manipulator not initialized");
        return false;
    }
    
    // Publish zero velocity command
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
    
    cmd_vel_pub_->publish(cmd);
    
    // Update state
    base_moving_ = false;
    has_base_target_ = false;
    
    RCLCPP_INFO(logger_, "Base movement stopped");
    
    return true;
}

bool MobileManipulator::setJointPositions(const std::vector<double>& joint_positions) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "Mobile manipulator not initialized");
        return false;
    }
    
#ifdef HAVE_OPEN_MANIPULATOR
    try {
        if (joint_positions.size() != 4) {
            RCLCPP_ERROR(logger_, "Expected 4 joint positions for OpenMANIPULATOR-X, got %zu", 
                        joint_positions.size());
            return false;
        }
        
        // Set joint targets using MoveIt
        move_group_arm_->setJointValueTarget({
            joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3]
        });
        
        // Create a plan
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_arm_->plan(plan));
        
        if (success) {
            // Execute the plan
            move_group_arm_->execute(plan);
            
            // Store target positions
            target_joint_positions_ = joint_positions;
            has_joint_target_ = true;
            manipulator_moving_ = true;
            
            RCLCPP_INFO(logger_, "Setting joint positions to [%.2f, %.2f, %.2f, %.2f]", 
                     joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3]);
                     
            return true;
        } else {
            RCLCPP_ERROR(logger_, "Failed to plan trajectory for joint positions");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "MoveIt error: %s", e.what());
        return false;
    }
#else
    // Simulation of success for when MoveIt is not available
    // Store target positions
    target_joint_positions_ = joint_positions;
    has_joint_target_ = true;
    manipulator_moving_ = true;
    
    RCLCPP_INFO(logger_, "Simulation: Setting joint positions to [%.2f, %.2f, %.2f, %.2f]", 
             joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3]);
             
    return true;
#endif
}

bbool MobileManipulator::setEndEffectorPose(const geometry_msgs::msg::Pose& pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "Mobile manipulator not initialized");
        return false;
    }
    
#ifdef HAVE_OPEN_MANIPULATOR
    try {
        // Set pose target using MoveIt
        move_group_arm_->setPoseTarget(pose);
        
        // Create a plan
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_arm_->plan(plan));
        
        if (success) {
            // Execute the plan
            move_group_arm_->execute(plan);
            
            // Store target pose
            target_ee_pose_ = pose;
            has_ee_target_ = true;
            manipulator_moving_ = true;
            
            RCLCPP_INFO(logger_, "Setting end effector pose to [%.2f, %.2f, %.2f]", 
                     pose.position.x, pose.position.y, pose.position.z);
                     
            return true;
        } else {
            RCLCPP_ERROR(logger_, "Failed to plan trajectory for end effector pose");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "MoveIt error: %s", e.what());
        return false;
    }
#else
    // Simulation of success for when MoveIt is not available
    // Store target pose
    target_ee_pose_ = pose;
    has_ee_target_ = true;
    manipulator_moving_ = true;
    
    RCLCPP_INFO(logger_, "Simulation: Setting end effector pose to [%.2f, %.2f, %.2f]", 
             pose.position.x, pose.position.y, pose.position.z);
             
    return true;
#endif
}

bool MobileManipulator::executeTrajectory(const std::vector<geometry_msgs::msg::Pose>& waypoints) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_ || waypoints.empty()) {
        RCLCPP_ERROR(logger_, "Mobile manipulator not initialized or empty waypoints");
        return false;
    }
    
    // For now, just execute the first waypoint
    // In a real implementation, this would handle the waypoints sequentially
    return setEndEffectorPose(waypoints[0]);
}

bool MobileManipulator::controlGripper(bool open) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "Mobile manipulator not initialized");
        return false;
    }
    
#ifdef HAVE_OPEN_MANIPULATOR
    try {
        // Set gripper position using MoveIt
        // Typical joint limits for OpenManipulator gripper: [0.0, 0.01]
        double position = open ? 0.01 : 0.0;
        
        move_group_gripper_->setJointValueTarget("gripper", position);
        
        // Create a plan
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = static_cast<bool>(move_group_gripper_->plan(plan));
        
        if (success) {
            // Execute the plan
            move_group_gripper_->execute(plan);
            
            RCLCPP_INFO(logger_, "Setting gripper to %s", open ? "open" : "close");
            
            return true;
        } else {
            RCLCPP_ERROR(logger_, "Failed to plan trajectory for gripper");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "MoveIt error: %s", e.what());
        return false;
    }
#else
    // Simulation of success for when MoveIt is not available
    RCLCPP_INFO(logger_, "Simulation: Setting gripper to %s", open ? "open" : "close");
    
    return true;
#endif
}

geometry_msgs::msg::Pose MobileManipulator::getBasePose() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_base_pose_;
}

std::vector<double> MobileManipulator::getJointPositions() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<double> positions;
    
    // Extract current joint positions from joint state message
    if (!current_joint_state_.name.empty()) {
        // Find indices of manipulator joints
        std::vector<size_t> joint_indices;
        
        for (size_t i = 0; i < current_joint_state_.name.size(); ++i) {
            // Look for joints named "joint1", "joint2", etc.
            if (current_joint_state_.name[i].find("joint") != std::string::npos) {
                joint_indices.push_back(i);
                
                // Add joint position if valid
                if (i < current_joint_state_.position.size()) {
                    positions.push_back(current_joint_state_.position[i]);
                }
            }
        }
    }
    
    // If no valid joints found, return empty vector
    return positions;
}

geometry_msgs::msg::Pose MobileManipulator::getEndEffectorPose() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // In a real implementation, this would use forward kinematics or TF2
    // For now, return the target pose if available, otherwise identity pose
    
    if (has_ee_target_) {
        return target_ee_pose_;
    } else {
        geometry_msgs::msg::Pose identity_pose;
        identity_pose.position.x = 0.0;
        identity_pose.position.y = 0.0;
        identity_pose.position.z = 0.0;
        identity_pose.orientation.x = 0.0;
        identity_pose.orientation.y = 0.0;
        identity_pose.orientation.z = 0.0;
        identity_pose.orientation.w = 1.0;
        
        return identity_pose;
    }
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
        geometry_msgs::msg::Twist cmd = calculateVelocityCommand(target_base_pose_);
        
        // Publish command
        cmd_vel_pub_->publish(cmd);
        
        // Check if target reached
        double dx = target_base_pose_.position.x - current_base_pose_.position.x;
        double dy = target_base_pose_.position.y - current_base_pose_.position.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        // Get orientation error
        std::vector<double> target_euler = quaternionToEuler(target_base_pose_.orientation);
        std::vector<double> current_euler = quaternionToEuler(current_base_pose_.orientation);
        double angle_error = std::fabs(target_euler[2] - current_euler[2]);
        
        // Normalize angle error to [0, pi]
        angle_error = std::fmod(angle_error + M_PI, 2.0 * M_PI) - M_PI;
        if (angle_error < 0) angle_error = -angle_error;
        
        // Position and orientation tolerance
        double position_tolerance = 0.05;  // 5 cm
        double orientation_tolerance = 0.1;  // ~5.7 degrees
        
        // Get these from parameters if available
        if (params_.find("position_tolerance") != params_.end()) {
            position_tolerance = params_.at("position_tolerance");
        }
        
        if (params_.find("orientation_tolerance") != params_.end()) {
            orientation_tolerance = params_.at("orientation_tolerance");
        }
        
        // Check if target reached
        if (distance < position_tolerance && angle_error < orientation_tolerance) {
            // Target reached, stop movement
            geometry_msgs::msg::Twist zero_cmd;
            cmd_vel_pub_->publish(zero_cmd);
            
            base_moving_ = false;
            has_base_target_ = false;
            
            RCLCPP_INFO(logger_, "Base reached target position");
        }
    }
    
    // Update manipulator status
    if (manipulator_moving_) {
        // In a real implementation, this would check joint states or use service calls
        // For this simplified version, just simulate completion after a delay
        
        static int update_count = 0;
        update_count++;
        
        // Simulate completion after ~2 seconds (assuming 20 Hz update rate)
        if (update_count >= 40) {
            manipulator_moving_ = false;
            update_count = 0;
            
            RCLCPP_INFO(logger_, "Manipulator movement completed");
        }
    }
}

void MobileManipulator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Update current pose and velocity
    current_base_pose_ = msg->pose.pose;
    current_velocity_ = msg->twist.twist;
}

void MobileManipulator::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Update current joint state
    current_joint_state_ = *msg;
}

#ifdef HAVE_OPEN_MANIPULATOR
void MobileManipulator::manipulatorStateCallback(
    const open_manipulator_msgs::msg::OpenManipulatorState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Update manipulator state
    manipulator_state_ = msg->open_manipulator_moving_state;
    
    // Update moving flag based on state
    manipulator_moving_ = (manipulator_state_ == "MOVING");
}
#endif

geometry_msgs::msg::Twist MobileManipulator::calculateVelocityCommand(
    const geometry_msgs::msg::Pose& target_pose) {
    
    // Calculate error
    double dx = target_pose.position.x - current_base_pose_.position.x;
    double dy = target_pose.position.y - current_base_pose_.position.y;
    
    // Calculate distance and target heading
    double distance = std::sqrt(dx*dx + dy*dy);
    double target_heading = std::atan2(dy, dx);
    
    // Get current orientation as euler angles
    std::vector<double> current_euler = quaternionToEuler(current_base_pose_.orientation);
    double current_yaw = current_euler[2];
    
    // Calculate heading error
    double heading_error = target_heading - current_yaw;
    
    // Normalize heading error to [-pi, pi]
    heading_error = std::fmod(heading_error + M_PI, 2.0 * M_PI) - M_PI;
    
    // Get time since last update
    auto current_time = node_->now();
    double dt = (current_time - last_error_time_).seconds();
    if (dt <= 0.0) dt = 0.01;  // Prevent division by zero
    
    // Limit dt to prevent huge jumps
    dt = std::min(dt, 0.1);
    
    // Linear velocity control (PID)
    double linear_error = distance;
    double linear_error_rate = (linear_error - prev_linear_error_) / dt;
    linear_error_sum_ += linear_error * dt;
    
    // Anti-windup for linear
    double max_sum = 1.0 / linear_ki_;
    if (linear_ki_ > 0) {
        linear_error_sum_ = std::max(-max_sum, std::min(linear_error_sum_, max_sum));
    }
    
    double linear_command = linear_kp_ * linear_error + 
                           linear_ki_ * linear_error_sum_ + 
                           linear_kd_ * linear_error_rate;
    
    // Angular velocity control (PID)
    double angular_error = heading_error;
    double angular_error_rate = (angular_error - prev_angular_error_) / dt;
    angular_error_sum_ += angular_error * dt;
    
    // Anti-windup for angular
    double max_angular_sum = 1.0 / angular_ki_;
    if (angular_ki_ > 0) {
        angular_error_sum_ = std::max(-max_angular_sum, std::min(angular_error_sum_, max_angular_sum));
    }
    
    double angular_command = angular_kp_ * angular_error + 
                            angular_ki_ * angular_error_sum_ + 
                            angular_kd_ * angular_error_rate;
    
    // Create velocity command
    geometry_msgs::msg::Twist cmd;
    
    // Limit velocity based on distance to target
    double max_linear_vel = 0.26;  // m/s - from TurtleBot3 specs
    double max_angular_vel = 1.82;  // rad/s - from TurtleBot3 specs
    
    // Slow down when approaching target
    double slow_down_distance = 0.5;  // m
    double slow_down_factor = std::min(1.0, distance / slow_down_distance);
    
    // Limit linear velocity based on angular error
    double heading_factor = std::max(0.1, 1.0 - std::fabs(heading_error) / M_PI);
    
    // Apply velocity limits
    linear_command = std::max(-max_linear_vel, std::min(linear_command, max_linear_vel));
    linear_command *= slow_down_factor * heading_factor;
    
    angular_command = std::max(-max_angular_vel, std::min(angular_command, max_angular_vel));
    
    // Set commands
    cmd.linear.x = linear_command;
    cmd.angular.z = angular_command;
    
    // Update previous errors
    prev_linear_error_ = linear_error;
    prev_angular_error_ = angular_error;
    last_error_time_ = current_time;
    
    return cmd;
}

std::vector<double> MobileManipulator::quaternionToEuler(const geometry_msgs::msg::Quaternion& quat) const {
    tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    
    return {roll, pitch, yaw};
}

} // namespace robot
} // namespace decentralized_auction_mm