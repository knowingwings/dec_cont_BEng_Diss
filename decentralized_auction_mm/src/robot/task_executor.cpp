#include "decentralized_auction_mm/robot/task_executor.hpp"
#include <cmath>

namespace decentralized_auction_mm {
namespace robot {

TaskExecutor::TaskExecutor(
    uint32_t robot_id,
    rclcpp::Node* node,
    MobileManipulator* mobile_manipulator,
    const std::map<std::string, double>& params)
    : robot_id_(robot_id),
      node_(node),
      mobile_manipulator_(mobile_manipulator),
      params_(params),
      state_(ExecutionState::IDLE),
      progress_(0.0),
      task_completed_(false),
      current_waypoint_(0),
      is_leader_(false),
      sync_received_(false),
      logger_(node->get_logger())
{
    RCLCPP_INFO(logger_, "Task Executor initialized for robot %d", robot_id);
}

bool TaskExecutor::startTask(const msg::Task& task) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (state_ != ExecutionState::IDLE) {
        RCLCPP_ERROR(logger_, "Task executor already busy with another task");
        return false;
    }
    
    // Check if this robot can execute the task
    if (!canExecuteTask(task)) {
        RCLCPP_ERROR(logger_, "Robot cannot execute this task");
        return false;
    }
    
    // Store task
    current_task_ = task;
    
    // For collaborative tasks, determine if this robot is the leader
    if (task.is_collaborative) {
        is_leader_ = (robot_id_ == 1);  // Arbitrary leadership assignment, could be more sophisticated
        sync_received_ = false;
        
        RCLCPP_INFO(logger_, "Robot %d is %s for collaborative task %d",
                 robot_id_, is_leader_ ? "leader" : "follower", task.id);
    }
    
    // Plan task execution
    if (!planTaskExecution(task)) {
        RCLCPP_ERROR(logger_, "Failed to plan task execution");
        return false;
    }
    
    // Set initial state
    state_ = ExecutionState::MOVING_TO_TASK;
    progress_ = 0.0;
    task_completed_ = false;
    current_waypoint_ = 0;
    task_start_time_ = node_->now();
    
    RCLCPP_INFO(logger_, "Starting task %d execution", task.id);
    
    return true;
}

double TaskExecutor::update() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // If not executing, return -1
    if (state_ == ExecutionState::IDLE) {
        return -1.0;
    }
    
    // Update progress based on state
    switch (state_) {
        case ExecutionState::MOVING_TO_TASK:
            // Check if base has reached target position
            if (!mobile_manipulator_->isBaseMoving()) {
                // Base has reached target position
                RCLCPP_INFO(logger_, "Reached task position");
                
                // Move to next state
                state_ = ExecutionState::CONFIGURING_MANIPULATOR;
                progress_ = 0.2;  // 20% progress
                
                // Configure manipulator
                configureManipulator();
            } else {
                // Still moving, update progress based on distance
                geometry_msgs::msg::Pose current_pose = mobile_manipulator_->getBasePose();
                double dx = current_pose.position.x - target_base_pose_.position.x;
                double dy = current_pose.position.y - target_base_pose_.position.y;
                double distance = std::sqrt(dx*dx + dy*dy);
                
                // Progress is inversely proportional to distance
                // (assuming initial distance was larger)
                progress_ = 0.2 * (1.0 - std::min(1.0, distance / 0.5));
            }
            break;
            
        case ExecutionState::CONFIGURING_MANIPULATOR:
            // Check if manipulator has reached target configuration
            if (!mobile_manipulator_->isManipulatorMoving()) {
                // Manipulator has reached target configuration
                RCLCPP_INFO(logger_, "Manipulator configured");
                
                // For collaborative tasks, handle synchronization
                if (current_task_.is_collaborative) {
                    state_ = ExecutionState::WAITING_FOR_SYNC;
                    progress_ = 0.3;  // 30% progress
                    
                    // Handle synchronization
                    handleSynchronization();
                } else {
                    // Move to next state
                    state_ = ExecutionState::EXECUTING_MANIPULATION;
                    progress_ = 0.4;  // 40% progress
                    
                    // Execute manipulation
                    executeManipulation();
                }
            } else {
                // Still configuring, progress between 20% and 30%
                progress_ = 0.2 + 0.1 * (1.0 - mobile_manipulator_->isManipulatorMoving());
            }
            break;
            
        case ExecutionState::WAITING_FOR_SYNC:
            // Check if synchronization is complete
            if (sync_received_) {
                // Synchronization complete
                RCLCPP_INFO(logger_, "Synchronization complete");
                
                // Move to next state
                state_ = ExecutionState::EXECUTING_MANIPULATION;
                progress_ = 0.4;  // 40% progress
                
                // Execute manipulation
                executeManipulation();
            } else {
                // Check for timeout
                auto current_time = node_->now();
                double elapsed = (current_time - sync_request_time_).seconds();
                
                if (elapsed > 5.0) {  // 5 second timeout
                    RCLCPP_WARN(logger_, "Synchronization timeout");
                    
                    // Retry synchronization
                    handleSynchronization();
                }
                
                // Progress stays at 30% while waiting
                progress_ = 0.3;
            }
            break;
            
        case ExecutionState::EXECUTING_MANIPULATION:
            // Check if manipulation is complete
            if (!mobile_manipulator_->isManipulatorMoving()) {
                // Check if there are more waypoints
                if (current_waypoint_ < manipulation_waypoints_.size() - 1) {
                    // Move to next waypoint
                    current_waypoint_++;
                    
                    // Execute next waypoint
                    mobile_manipulator_->setEndEffectorPose(manipulation_waypoints_[current_waypoint_]);
                    
                    // Update progress (40% to 90% based on waypoint progress)
                    progress_ = 0.4 + 0.5 * ((double)current_waypoint_ / (double)manipulation_waypoints_.size());
                } else {
                    // Manipulation complete
                    RCLCPP_INFO(logger_, "Manipulation complete");
                    
                    // Move to next state
                    state_ = ExecutionState::FINISHING;
                    progress_ = 0.9;  // 90% progress
                    
                    // Finish task
                    finishTask();
                }
            } else {
                // Still executing manipulation, progress between 40% and 90%
                double waypoint_progress = (double)current_waypoint_ / (double)manipulation_waypoints_.size();
                progress_ = 0.4 + 0.5 * waypoint_progress;
            }
            break;
            
        case ExecutionState::FINISHING:
            // Check if finishing is complete
            if (!mobile_manipulator_->isManipulatorMoving() && !mobile_manipulator_->isBaseMoving()) {
                // Task complete
                RCLCPP_INFO(logger_, "Task %d completed", current_task_.id);
                
                // Set final state
                state_ = ExecutionState::IDLE;
                progress_ = 1.0;  // 100% progress
                task_completed_ = true;
                
                // Return -1 to indicate completion
                return -1.0;
            } else {
                // Still finishing, progress between 90% and 100%
                progress_ = 0.9 + 0.1 * (1.0 - mobile_manipulator_->isManipulatorMoving());
            }
            break;
            
        default:
            // Should not happen
            RCLCPP_ERROR(logger_, "Unknown execution state");
            break;
    }
    
    return progress_;
}

bool TaskExecutor::isExecuting() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_ != ExecutionState::IDLE;
}

msg::Task TaskExecutor::getCurrentTask() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_task_;
}

bool TaskExecutor::cancelTask() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (state_ == ExecutionState::IDLE) {
        return false;
    }
    
    // Stop all movement
    mobile_manipulator_->stopBase();
    
    // Reset state
    state_ = ExecutionState::IDLE;
    progress_ = 0.0;
    task_completed_ = false;
    
    RCLCPP_INFO(logger_, "Task execution cancelled");
    
    return true;
}

bool TaskExecutor::canExecuteTask(const msg::Task& task) const {
    // For collaborative tasks, always return true (assuming both robots are needed)
    if (task.is_collaborative) {
        return true;
    }
    
    // Check robot capabilities against task requirements
    // This is a simple check - in a real implementation, this would be more sophisticated
    return true;
}

bool TaskExecutor::planTaskExecution(const msg::Task& task) {
    // Plan base movement
    target_base_pose_.position = task.position.position;
    target_base_pose_.orientation = task.position.orientation;
    
    // Plan manipulator configuration
    // For simplicity, use a pre-defined configuration based on task type
    target_joint_positions_ = {0.0, -0.5, 0.5, 0.0};  // Example configuration
    
    // Plan manipulation waypoints
    manipulation_waypoints_.clear();
    
    // Add waypoints for a simple manipulation task
    geometry_msgs::msg::Pose waypoint1;
    waypoint1.position.x = task.position.position.x;
    waypoint1.position.y = task.position.position.y;
    waypoint1.position.z = task.position.position.z + 0.1;  // Above task
    waypoint1.orientation = task.position.orientation;
    
    geometry_msgs::msg::Pose waypoint2;
    waypoint2.position.x = task.position.position.x;
    waypoint2.position.y = task.position.position.y;
    waypoint2.position.z = task.position.position.z;  // At task
    waypoint2.orientation = task.position.orientation;
    
    geometry_msgs::msg::Pose waypoint3;
    waypoint3.position.x = task.position.position.x;
    waypoint3.position.y = task.position.position.y;
    waypoint3.position.z = task.position.position.z + 0.1;  // Above task again
    waypoint3.orientation = task.position.orientation;
    
    manipulation_waypoints_.push_back(waypoint1);
    manipulation_waypoints_.push_back(waypoint2);
    manipulation_waypoints_.push_back(waypoint3);
    
    return true;
}

bool TaskExecutor::moveBaseToTask() {
    return mobile_manipulator_->moveBase(target_base_pose_);
}

bool TaskExecutor::configureManipulator() {
    return mobile_manipulator_->setJointPositions(target_joint_positions_);
}

bool TaskExecutor::executeManipulation() {
    if (manipulation_waypoints_.empty()) {
        RCLCPP_ERROR(logger_, "No manipulation waypoints");
        return false;
    }
    
    // Move to first waypoint
    current_waypoint_ = 0;
    return mobile_manipulator_->setEndEffectorPose(manipulation_waypoints_[current_waypoint_]);
}

bool TaskExecutor::finishTask() {
    // Move manipulator to home position
    std::vector<double> home_position = {0.0, 0.0, 0.0, 0.0};
    return mobile_manipulator_->setJointPositions(home_position);
}

bool TaskExecutor::handleSynchronization() {
    // In a real implementation, this would involve communication between robots
    // For this simulation, we'll just simulate synchronization
    
    if (is_leader_) {
        // Leader sends synchronization signal
        RCLCPP_INFO(logger_, "Leader sending synchronization signal");
        
        // In a real implementation, this would publish a synchronization message
        
        // For simulation, just assume follower receives it immediately
        sync_received_ = true;
    } else {
        // Follower waits for synchronization signal
        RCLCPP_INFO(logger_, "Follower waiting for synchronization signal");
        
        // In a real implementation, this would wait for a synchronization message
        
        // For simulation, just simulate a delay
        sync_request_time_ = node_->now();
        
        // After a short delay, simulate receiving synchronization
        auto timer = node_->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]() {
                sync_received_ = true;
                RCLCPP_INFO(logger_, "Follower received synchronization signal");
            });
        
        // Only execute once
        timer->cancel();
    }
    
    return true;
}

} // namespace robot
} // namespace decentralized_auction_mm