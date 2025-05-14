#include "decentralized_auction_mm/robot/robot_controller.hpp"
#include <functional>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace decentralized_auction_mm {
namespace robot {

RobotController::RobotController(
    uint32_t robot_id,
    rclcpp::Node* node,
    const std::map<std::string, double>& params)
    : robot_id_(robot_id),
      node_(node),
      params_(params),
      initialized_(false),
      failed_(false),
      executing_task_(false),
      task_progress_(0.0),
      heartbeat_(0),
      logger_(node->get_logger())
{
    // Initialize capability vector
    capabilities_ = {1.0, 1.0, 1.0, 1.0, 1.0};  // Default balanced capabilities
    
    // Set capabilities based on robot ID for demonstration
    if (robot_id == 1) {
        // Robot 1 is better at precision tasks
        capabilities_ = {1.2, 0.8, 1.1, 0.9, 1.0};
    } else if (robot_id == 2) {
        // Robot 2 is better at force tasks
        capabilities_ = {0.8, 1.2, 0.9, 1.1, 1.0};
    }
    
    RCLCPP_INFO(logger_, "Robot controller initialized for robot %d", robot_id);
}

void RobotController::initialize() {
    // Create TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Create subscribers
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&RobotController::odomCallback, this, std::placeholders::_1));
    
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&RobotController::jointStateCallback, this, std::placeholders::_1));
    
    // Create publishers
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Initialize mobile manipulator
    mobile_manipulator_ = std::make_unique<MobileManipulator>(robot_id_, node_, params_);
    mobile_manipulator_->initialize();
    
    // Initialize task executor
    task_executor_ = std::make_unique<TaskExecutor>(robot_id_, node_, mobile_manipulator_.get(), params_);
    
    initialized_ = true;
    RCLCPP_INFO(logger_, "Robot controller fully initialized");
}

bool RobotController::assignTask(const msg::Task& task) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "Cannot assign task: robot controller not initialized");
        return false;
    }
    
    if (failed_) {
        RCLCPP_ERROR(logger_, "Cannot assign task: robot has failed");
        return false;
    }
    
    // If already executing a task, queue the new one
    if (executing_task_) {
        task_queue_.push(task);
        RCLCPP_INFO(logger_, "Task %d queued for later execution", task.id);
        return true;
    }
    
    // Assign task to executor
    bool success = task_executor_->startTask(task);
    
    if (success) {
        current_task_ = task;
        executing_task_ = true;
        task_progress_ = 0.0;
        RCLCPP_INFO(logger_, "Task %d assigned for execution", task.id);
    } else {
        RCLCPP_ERROR(logger_, "Failed to assign task %d", task.id);
    }
    
    return success;
}

bool RobotController::update() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_ || failed_) {
        return false;
    }
    
    // Update mobile manipulator
    mobile_manipulator_->update();
    
    // Update task executor if executing
    if (executing_task_) {
        task_progress_ = task_executor_->update();
        
        // Check if task completed or failed
        if (task_progress_ < 0.0) {
            // Task finished or failed
            executing_task_ = false;
            
            // Update current task status
            current_task_.progress = 1.0;
            current_task_.status = msg::Task::STATUS_COMPLETED;
            
            RCLCPP_INFO(logger_, "Task %d completed", current_task_.id);
            
            // Start next task if any
            if (!task_queue_.empty()) {
                msg::Task next_task = task_queue_.front();
                task_queue_.pop();
                
                bool success = task_executor_->startTask(next_task);
                
                if (success) {
                    current_task_ = next_task;
                    executing_task_ = true;
                    task_progress_ = 0.0;
                    RCLCPP_INFO(logger_, "Starting next task %d from queue", next_task.id);
                } else {
                    RCLCPP_ERROR(logger_, "Failed to start next task %d from queue", next_task.id);
                }
            }
        }
    }
    
    // Increment heartbeat
    heartbeat_++;
    
    return executing_task_;
}

msg::RobotStatus RobotController::getRobotStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    msg::RobotStatus status;
    status.header.stamp = node_->now();
    status.robot_id = robot_id_;
    status.pose = current_pose_;
    status.capabilities = capabilities_;
    status.current_workload = getWorkload();
    status.failed = failed_;
    status.heartbeat = heartbeat_;
    
    // Add assigned tasks
    if (executing_task_) {
        status.assigned_tasks.push_back(current_task_.id);
    }
    
    // Add queued tasks
    std::queue<msg::Task> temp_queue = task_queue_;
    while (!temp_queue.empty()) {
        status.assigned_tasks.push_back(temp_queue.front().id);
        temp_queue.pop();
    }
    
    return status;
}

void RobotController::setFailed(bool failed) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!failed_ && failed) {
        RCLCPP_ERROR(logger_, "Robot %d set to failed state", robot_id_);
    } else if (failed_ && !failed) {
        RCLCPP_INFO(logger_, "Robot %d recovered from failure", robot_id_);
    }
    
    failed_ = failed;
    
    // If failed, stop any current task
    if (failed_ && executing_task_) {
        task_executor_->cancelTask();
        executing_task_ = false;
    }
}

bool RobotController::hasFailed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return failed_;
}

geometry_msgs::msg::Point RobotController::getPosition() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_pose_.position;
}

std::vector<double> RobotController::getCapabilities() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return capabilities_;
}

double RobotController::getWorkload() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    double workload = 0.0;
    
    // Add current task workload
    if (executing_task_) {
        workload += current_task_.execution_time * (1.0 - task_progress_);
    }
    
    // Add queued tasks workload
    std::queue<msg::Task> temp_queue = task_queue_;
    while (!temp_queue.empty()) {
        workload += temp_queue.front().execution_time;
        temp_queue.pop();
    }
    
    return workload;
}

std::vector<uint32_t> RobotController::getAssignedTasks() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<uint32_t> assigned_tasks;
    
    // Add current task
    if (executing_task_) {
        assigned_tasks.push_back(current_task_.id);
    }
    
    // Add queued tasks
    std::queue<msg::Task> temp_queue = task_queue_;
    while (!temp_queue.empty()) {
        assigned_tasks.push_back(temp_queue.front().id);
        temp_queue.pop();
    }
    
    return assigned_tasks;
}

void RobotController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_pose_ = msg->pose.pose;
}

void RobotController::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_joint_state_ = *msg;
}

bool RobotController::moveToPosition(const geometry_msgs::msg::Point& target) {
    if (!initialized_ || failed_) {
        return false;
    }
    
    // Create target pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position = target;
    target_pose.orientation = current_pose_.orientation;
    
    return mobile_manipulator_->moveBase(target_pose);
}

bool RobotController::configureManipulator(const msg::Task& task) {
    if (!initialized_ || failed_) {
        return false;
    }
    
    // For simplicity, use a default configuration
    std::vector<double> joint_positions = {0.0, -0.5, 0.5, 0.0};
    
    return mobile_manipulator_->setJointPositions(joint_positions);
}

double RobotController::updateTaskProgress() {
    if (!executing_task_) {
        return -1.0;
    }
    
    // Simulate progress update
    task_progress_ += 0.01;
    
    if (task_progress_ >= 1.0) {
        task_progress_ = 1.0;
        return -1.0;  // Indicate completion
    }
    
    return task_progress_;
}

} // namespace robot
} // namespace decentralized_auction_mm