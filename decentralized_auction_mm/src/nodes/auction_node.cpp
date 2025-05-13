#include <memory>
#include <string>
#include <vector>
#include <map>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "decentralized_auction_mm/msg/task.hpp"
#include "decentralized_auction_mm/msg/task_array.hpp"
#include "decentralized_auction_mm/msg/bid.hpp"
#include "decentralized_auction_mm/msg/bid_array.hpp"
#include "decentralized_auction_mm/msg/robot_status.hpp"
#include "decentralized_auction_mm/msg/auction_status.hpp"

#include "decentralized_auction_mm/srv/start_auction.hpp"
#include "decentralized_auction_mm/srv/stop_auction.hpp"
#include "decentralized_auction_mm/srv/fail_robot.hpp"

using namespace std::chrono_literals;

namespace decentralized_auction_mm {

/**
 * @class AuctionNode
 * @brief Main node for managing the auction process
 * 
 * This node monitors the auction process, maintains global auction state,
 * publishes auction status, and provides services for controlling the auction.
 */
class AuctionNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     */
    AuctionNode() : Node("auction_node") {
        // Declare and get parameters
        this->declare_parameter("auction_status_rate", 1.0);
        this->declare_parameter("task_persistence", true);
        this->declare_parameter("fail_robot_id", 0);
        this->declare_parameter("fail_time", 0.0);
        this->declare_parameter("epsilon", 0.05);
        
        auction_status_rate_ = this->get_parameter("auction_status_rate").as_double();
        task_persistence_ = this->get_parameter("task_persistence").as_bool();
        fail_robot_id_ = this->get_parameter("fail_robot_id").as_int();
        fail_time_ = this->get_parameter("fail_time").as_double();
        epsilon_ = this->get_parameter("epsilon").as_double();
        
        // Create publishers
        task_pub_ = this->create_publisher<msg::TaskArray>("tasks", 10);
        auction_status_pub_ = this->create_publisher<msg::AuctionStatus>("auction_status", 10);
        visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization/auction", 10);
        
        // Create subscribers
        robot_status_sub_ = this->create_subscription<msg::RobotStatus>(
            "robot_status", 10, std::bind(&AuctionNode::robotStatusCallback, this, std::placeholders::_1));
        
        bid_sub_ = this->create_subscription<msg::Bid>(
            "bids", 10, std::bind(&AuctionNode::bidCallback, this, std::placeholders::_1));
        
        task_sub_ = this->create_subscription<msg::TaskArray>(
            "task_updates", 10, std::bind(&AuctionNode::taskUpdateCallback, this, std::placeholders::_1));
        
        // Create services
        start_service_ = this->create_service<srv::StartAuction>(
            "start_auction", std::bind(&AuctionNode::startAuctionCallback, this, 
                       std::placeholders::_1, std::placeholders::_2));
        
        stop_service_ = this->create_service<srv::StopAuction>(
            "stop_auction", std::bind(&AuctionNode::stopAuctionCallback, this, 
                       std::placeholders::_1, std::placeholders::_2));
        
        fail_robot_service_ = this->create_service<srv::FailRobot>(
            "fail_robot", std::bind(&AuctionNode::failRobotCallback, this, 
                       std::placeholders::_1, std::placeholders::_2));
        
        // Create timers
        status_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / auction_status_rate_),
            std::bind(&AuctionNode::publishAuctionStatus, this));
        
        // Set up failure simulation if requested
        if (fail_robot_id_ > 0 && fail_time_ > 0.0) {
            RCLCPP_INFO(this->get_logger(), "Will simulate failure of robot %d after %.1f seconds",
                     fail_robot_id_, fail_time_);
            
            fail_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(fail_time_),
                std::bind(&AuctionNode::triggerFailure, this));
        }
        
        // Initialize auction status
        auction_status_.header.stamp = this->now();
        auction_status_.status = msg::AuctionStatus::STATUS_INACTIVE;
        auction_status_.iteration = 0;
        auction_status_.total_tasks = 0;
        auction_status_.assigned_tasks = 0;
        auction_status_.completed_tasks = 0;
        auction_status_.recovery_iteration = 0;
        auction_status_.failed_robot_id = 0;
        auction_status_.global_makespan = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Auction Node initialized");
    }

private:
    /**
     * @brief Callback for robot status messages
     * @param msg The robot status message
     */
    void robotStatusCallback(const msg::RobotStatus::SharedPtr msg) {
        // Store robot status
        robot_status_[msg->robot_id] = *msg;
        last_heartbeat_[msg->robot_id] = this->now();
        
        // Check for robot failures
        if (msg->failed && auction_status_.failed_robot_id == 0) {
            RCLCPP_WARN(this->get_logger(), "Detected failure of robot %d", msg->robot_id);
            auction_status_.failed_robot_id = msg->robot_id;
            auction_status_.status = msg::AuctionStatus::STATUS_RECOVERY;
            auction_status_.recovery_iteration = 0;
        }
        
        // Update task status based on robot's assigned tasks
        if (!task_persistence_) {
            return; // Skip if tasks are managed externally
        }
        
        // Update assigned tasks status
        std::vector<uint32_t> updated_tasks;
        for (auto task_id : msg->assigned_tasks) {
            // Find the task
            for (auto& task : tasks_) {
                if (task.id == task_id) {
                    // Update assignment if different
                    if (task.assigned_robot != msg->robot_id) {
                        task.assigned_robot = msg->robot_id;
                        task.status = msg::Task::STATUS_ASSIGNED;
                        updated_tasks.push_back(task_id);
                    }
                    break;
                }
            }
        }
        
        // Republish tasks if any were updated
        if (!updated_tasks.empty()) {
            publishTasks();
        }
    }
    
    /**
     * @brief Callback for bid messages
     * @param msg The bid message
     */
    void bidCallback(const msg::Bid::SharedPtr msg) {
        // Store bid for visualization and analysis
        recent_bids_.push_back(*msg);
        
        // Limit the number of recent bids stored
        if (recent_bids_.size() > 100) {
            recent_bids_.erase(recent_bids_.begin());
        }
    }
    
    /**
     * @brief Callback for task update messages
     * @param msg The task array message
     */
    void taskUpdateCallback(const msg::TaskArray::SharedPtr msg) {
        if (!task_persistence_) {
            return; // Skip if tasks are managed externally
        }
        
        // Update tasks
        for (const auto& update_task : msg->tasks) {
            bool found = false;
            
            // Find and update existing task
            for (auto& task : tasks_) {
                if (task.id == update_task.id) {
                    // Update task status
                    task.status = update_task.status;
                    task.progress = update_task.progress;
                    task.assigned_robot = update_task.assigned_robot;
                    task.current_price = update_task.current_price;
                    found = true;
                    break;
                }
            }
            
            // Add new task if not found
            if (!found) {
                tasks_.push_back(update_task);
            }
        }
        
        // Update auction status
        updateAuctionStatus();
        
        // Republish all tasks to ensure consistency
        publishTasks();
    }
    
    /**
     * @brief Service callback to start the auction
     * @param request The service request
     * @param response The service response
     */
    void startAuctionCallback(
        const std::shared_ptr<srv::StartAuction::Request>,
        std::shared_ptr<srv::StartAuction::Response> response)
    {
        if (auction_status_.status == msg::AuctionStatus::STATUS_INACTIVE) {
            auction_status_.status = msg::AuctionStatus::STATUS_RUNNING;
            auction_status_.iteration = 0;
            auction_start_time_ = this->now();
            
            RCLCPP_INFO(this->get_logger(), "Auction started");
            response->success = true;
            response->message = "Auction started successfully";
        } else {
            RCLCPP_WARN(this->get_logger(), "Auction already running");
            response->success = false;
            response->message = "Auction is already running";
        }
    }
    
    /**
     * @brief Service callback to stop the auction
     * @param request The service request
     * @param response The service response
     */
    void stopAuctionCallback(
        const std::shared_ptr<srv::StopAuction::Request>,
        std::shared_ptr<srv::StopAuction::Response> response)
    {
        if (auction_status_.status == msg::AuctionStatus::STATUS_RUNNING ||
            auction_status_.status == msg::AuctionStatus::STATUS_RECOVERY) {
            
            auction_status_.status = msg::AuctionStatus::STATUS_COMPLETED;
            
            RCLCPP_INFO(this->get_logger(), "Auction stopped");
            response->success = true;
            response->message = "Auction stopped successfully";
        } else {
            RCLCPP_WARN(this->get_logger(), "No auction running to stop");
            response->success = false;
            response->message = "No auction is currently running";
        }
    }
    
    /**
     * @brief Service callback to simulate a robot failure
     * @param request The service request
     * @param response The service response
     */
    void failRobotCallback(
        const std::shared_ptr<srv::FailRobot::Request> request,
        std::shared_ptr<srv::FailRobot::Response> response)
    {
        if (auction_status_.status != msg::AuctionStatus::STATUS_RUNNING) {
            response->success = false;
            response->message = "Cannot fail robot: auction not running";
            return;
        }
        
        if (auction_status_.failed_robot_id > 0) {
            response->success = false;
            response->message = "Another robot is already in failure state";
            return;
        }
        
        uint32_t robot_id = request->robot_id;
        
        // Check if robot exists
        if (robot_status_.find(robot_id) == robot_status_.end()) {
            response->success = false;
            response->message = "Robot " + std::to_string(robot_id) + " not found";
            return;
        }
        
        // Set auction status to recovery
        auction_status_.status = msg::AuctionStatus::STATUS_RECOVERY;
        auction_status_.failed_robot_id = robot_id;
        auction_status_.recovery_iteration = 0;
        
        // Forward failure to the robot's failure service
        auto client = this->create_client<srv::FailRobot>(
            "robot" + std::to_string(robot_id) + "/fail_robot_" + std::to_string(robot_id));
        
        auto forward_request = std::make_shared<srv::FailRobot::Request>();
        forward_request->robot_id = robot_id;
        
        RCLCPP_INFO(this->get_logger(), "Simulating failure of robot %d", robot_id);
        
        // Send request asynchronously
        client->async_send_request(
            forward_request,
            [this, robot_id](rclcpp::Client<srv::FailRobot>::SharedFuture future) {
                auto result = future.get();
                if (result->success) {
                    RCLCPP_INFO(this->get_logger(), "Robot %d failure simulation successful", robot_id);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Robot %d failure simulation failed: %s", 
                             robot_id, result->message.c_str());
                }
            });
        
        response->success = true;
        response->message = "Failure of robot " + std::to_string(robot_id) + " initiated";
    }
    
    /**
     * @brief Trigger a robot failure (for simulation)
     */
    void triggerFailure() {
        // Cancel the timer to prevent repeated failures
        fail_timer_->cancel();
        
        // Create and send a failure request
        auto request = std::make_shared<srv::FailRobot::Request>();
        request->robot_id = fail_robot_id_;
        
        auto response = std::make_shared<srv::FailRobot::Response>();
        failRobotCallback(request, response);
    }
    
    /**
     * @brief Publish auction status
     */
    void publishAuctionStatus() {
        // Update auction status
        updateAuctionStatus();
        
        // Update timestamp
        auction_status_.header.stamp = this->now();
        
        // Increment iteration if auction is running
        if (auction_status_.status == msg::AuctionStatus::STATUS_RUNNING) {
            auction_status_.iteration++;
        } else if (auction_status_.status == msg::AuctionStatus::STATUS_RECOVERY) {
            auction_status_.iteration++;
            auction_status_.recovery_iteration++;
        }
        
        // Publish status
        auction_status_pub_->publish(auction_status_);
        
        // Check for missing heartbeats (robot failures)
        checkRobotHeartbeats();
    }
    
    /**
     * @brief Update auction status based on current state
     */
    void updateAuctionStatus() {
        if (!task_persistence_) {
            return; // Skip if tasks are managed externally
        }
        
        // Count tasks by status
        uint32_t total = tasks_.size();
        uint32_t assigned = 0;
        uint32_t completed = 0;
        
        for (const auto& task : tasks_) {
            if (task.assigned_robot > 0) {
                assigned++;
            }
            if (task.status == msg::Task::STATUS_COMPLETED) {
                completed++;
            }
        }
        
        auction_status_.total_tasks = total;
        auction_status_.assigned_tasks = assigned;
        auction_status_.completed_tasks = completed;
        
        // Calculate global makespan
        calculateGlobalMakespan();
        
        // Check for auction completion
        if (auction_status_.status == msg::AuctionStatus::STATUS_RUNNING &&
            completed == total && total > 0) {
            auction_status_.status = msg::AuctionStatus::STATUS_COMPLETED;
            RCLCPP_INFO(this->get_logger(), "Auction completed: all tasks finished");
        }
        
        // Check recovery completion
        if (auction_status_.status == msg::AuctionStatus::STATUS_RECOVERY) {
            bool all_reassigned = true;
            
            // Check if all tasks from failed robot have been reassigned
            for (const auto& task : tasks_) {
                if (task.assigned_robot == auction_status_.failed_robot_id &&
                    task.status != msg::Task::STATUS_COMPLETED) {
                    all_reassigned = false;
                    break;
                }
            }
            
            if (all_reassigned && auction_status_.recovery_iteration > 5) {
                RCLCPP_INFO(this->get_logger(), "Recovery completed after %d iterations",
                         auction_status_.recovery_iteration);
                auction_status_.status = msg::AuctionStatus::STATUS_RUNNING;
            }
        }
    }
    
    /**
     * @brief Calculate global makespan based on task assignments
     */
    void calculateGlobalMakespan() {
        std::map<uint32_t, double> robot_makespans;
        
        // Initialize makespans for all active robots
        for (const auto& [robot_id, status] : robot_status_) {
            if (!status.failed) {
                robot_makespans[robot_id] = 0.0;
            }
        }
        
        // Calculate makespan for each robot
        for (const auto& task : tasks_) {
            uint32_t robot_id = task.assigned_robot;
            if (robot_id > 0 && robot_makespans.find(robot_id) != robot_makespans.end() &&
                task.status != msg::Task::STATUS_COMPLETED) {
                robot_makespans[robot_id] += task.execution_time;
            }
        }
        
        // Global makespan is the maximum among robots
        double max_makespan = 0.0;
        for (const auto& [robot_id, makespan] : robot_makespans) {
            max_makespan = std::max(max_makespan, makespan);
        }
        
        auction_status_.global_makespan = max_makespan;
    }
    
    /**
     * @brief Check for missing robot heartbeats
     */
    void checkRobotHeartbeats() {
        // Check if any robot has missed heartbeats
        auto current_time = this->now();
        double heartbeat_timeout = 5.0; // seconds
        
        for (const auto& [robot_id, status] : robot_status_) {
            // Skip already failed robots
            if (status.failed) {
                continue;
            }
            
            // Check heartbeat
            auto last_time = last_heartbeat_[robot_id];
            double elapsed = (current_time - last_time).seconds();
            
            if (elapsed > heartbeat_timeout) {
                RCLCPP_WARN(this->get_logger(), "Robot %d heartbeat timeout (%.1f seconds)",
                         robot_id, elapsed);
                
                // If auction is running and no other robot has failed, trigger recovery
                if (auction_status_.status == msg::AuctionStatus::STATUS_RUNNING &&
                    auction_status_.failed_robot_id == 0) {
                    RCLCPP_ERROR(this->get_logger(), "Initiating recovery for robot %d failure", robot_id);
                    
                    auction_status_.status = msg::AuctionStatus::STATUS_RECOVERY;
                    auction_status_.failed_robot_id = robot_id;
                    auction_status_.recovery_iteration = 0;
                    
                    // Forward failure to the robot's failure service
                    auto client = this->create_client<srv::FailRobot>(
                        "robot" + std::to_string(robot_id) + "/fail_robot_" + std::to_string(robot_id));
                    
                    auto request = std::make_shared<srv::FailRobot::Request>();
                    request->robot_id = robot_id;
                    
                    // Send request asynchronously
                    client->async_send_request(request);
                }
            }
        }
    }
    
    /**
     * @brief Publish current tasks
     */
    void publishTasks() {
        if (!task_persistence_) {
            return; // Skip if tasks are managed externally
        }
        
        msg::TaskArray task_msg;
        task_msg.header.stamp = this->now();
        task_msg.tasks = tasks_;
        
        task_pub_->publish(task_msg);
    }

    // Parameters
    double auction_status_rate_;
    bool task_persistence_;
    uint32_t fail_robot_id_;
    double fail_time_;
    double epsilon_;
    
    // Auction state
    msg::AuctionStatus auction_status_;
    rclcpp::Time auction_start_time_;
    std::vector<msg::Task> tasks_;
    std::map<uint32_t, msg::RobotStatus> robot_status_;
    std::map<uint32_t, rclcpp::Time> last_heartbeat_;
    std::vector<msg::Bid> recent_bids_;
    
    // Publishers
    rclcpp::Publisher<msg::TaskArray>::SharedPtr task_pub_;
    rclcpp::Publisher<msg::AuctionStatus>::SharedPtr auction_status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
    
    // Subscribers
    rclcpp::Subscription<msg::RobotStatus>::SharedPtr robot_status_sub_;
    rclcpp::Subscription<msg::Bid>::SharedPtr bid_sub_;
    rclcpp::Subscription<msg::TaskArray>::SharedPtr task_sub_;
    
    // Services
    rclcpp::Service<srv::StartAuction>::SharedPtr start_service_;
    rclcpp::Service<srv::StopAuction>::SharedPtr stop_service_;
    rclcpp::Service<srv::FailRobot>::SharedPtr fail_robot_service_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr fail_timer_;
};

} // namespace decentralized_auction_mm

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<decentralized_auction_mm::AuctionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}