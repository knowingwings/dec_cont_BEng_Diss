#include "decentralized_auction_mm/robot/mobile_manipulator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
    
#ifdef HAVE_OPEN_MANIPULATOR
    manipulator_state_sub_ = node_->create_subscription<open_manipulator_msgs::msg::OpenManipulatorState>(
        "open_manipulator_x_controller/state", 10, 
        std::bind(&MobileManipulator::manipulatorStateCallback, this, std::placeholders::_1));
    
    // Create service clients
    set_joint_position_client_ = node_->create_client<open_manipulator_msgs::srv::SetJointPosition>(
        "open_manipulator_x_controller/set_joint_position");
    
    set_kinematics_pose_client_ = node_->create_client<open_manipulator_msgs::srv::SetKinematicsPose>(
        "open_manipulator_x_controller/set_kinematics_pose");
    
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
#endif
    
    // Create publishers
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Initialize TF2 listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    RCLCPP_INFO(logger_, "Mobile manipulator services connected");
    
    // Mark as initialized
    initialized_ = true;
    
    return true;
}

bool MobileManipulator::isBaseMoving() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return base_moving_;
}

bool MobileManipulator::isManipulatorMoving() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return manipulator_moving_;
}

bool MobileManipulator::setJointPositions(const std::vector<double>& joint_positions) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "Mobile manipulator not initialized");
        return false;
    }
    
#ifdef HAVE_OPEN_MANIPULATOR
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
#endif
    
    // Store target positions
    target_joint_positions_ = joint_positions;
    has_joint_target_ = true;
    manipulator_moving_ = true;
    
    RCLCPP_INFO(logger_, "Setting joint positions to [%.2f, %.2f, %.2f, %.2f]", 
             joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3]);
    
    return true;
}

// Rest of the implementation remains the same...

} // namespace robot
} // namespace decentralized_auction_mm