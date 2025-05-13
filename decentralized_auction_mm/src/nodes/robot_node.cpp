#include <memory>
#include <string>
#include <vector>
#include <map>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "decentralized_auction_mm/msg/task.hpp"
#include "decentralized_auction_mm/msg/task_array.hpp"
#include "decentralized_auction_mm/msg/bid.hpp"
#include "decentralized_auction_mm/msg/bid_array.hpp"
#include "decentralized_auction_mm/msg/robot_status.hpp"
#include "decentralized_auction_mm/msg/auction_status.hpp"
#include "decentralized_auction_mm/srv/fail_robot.hpp"

#include "decentralized_auction_mm/auction/auction_algorithm.hpp"
#include "decentralized_auction_mm/robot/robot_controller.hpp"
#include "decentralized_auction_mm/task/task_manager.hpp"

using namespace std::chrono_literals;

namespace decentralized_auction_mm {

/**
 * @class RobotNode
 * @brief Main node for running a robot in the decentralized auction system
 * 
 * This node integrates the auction algorithm, robot controller, and task
 * manager to participate in the distributed auction and execute tasks.
 */
class RobotNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     */
    RobotNode() : Node("robot_node") {
        // Declare and get parameters
        this->declare_parameter("robot_id", 1);
        this->declare_parameter("heartbeat_rate", 1.0);
        this->declare_parameter("auction_rate", 5.0);
        this->declare_parameter("controller_rate", 20.0);
        
        // Get parameters
        robot_id_ = this->get_parameter("robot_id").as_int();
        heartbeat_rate_ = this->get_parameter("heartbeat_rate").as_double();
        auction_rate_ = this->get_parameter("auction_rate").as_double();
        controller_rate_ = this->get_parameter("controller_rate").as_double();
        
        // Get auction parameters
        std::map<std::string, double> auction_params;
        this->declare_parameter("auction.epsilon", 0.05);
        auction_params["epsilon"] = this->get_parameter("auction.epsilon").as_double();
        
        for (int i = 1; i <= 5; i++) {
            std::string param_name = "auction.alpha" + std::to_string(i);
            this->declare_parameter(param_name, 1.0);
            auction_params["alpha" + std::to_string(i)] = 
                this->get_parameter(param_name).as_double();
        }
        
        this->declare_parameter("auction.gamma", 0.5);
        auction_params["gamma"] = this->get_parameter("auction.gamma").as_double();
        
        this->declare_parameter("auction.lambda", 0.1);
        auction_params["lambda"] = this->get_parameter("auction.lambda").as_double();
        
        for (int i = 1; i <= 3; i++) {
            std::string param_name = "auction.beta" + std::to_string(i);
            this->declare_parameter(param_name, 1.0);
            auction_params["beta" + std::to_string(i)] = 
                this->get_parameter(param_name).as_double();
        }
        
        // Get robot parameters
        std::map<std::string, double> robot_params;
        this->declare_parameter("robot.max_linear_vel", 0.26);
        robot_params["max_linear_vel"] = this->get_parameter("robot.max_linear_vel").as_double();
        
        this->declare_parameter("robot.max_angular_vel", 1.82);
        robot_params["max_angular_vel"] = this->get_parameter("robot.max_angular_vel").as_double();
        
        // Initialize components
        auction_algorithm_ = std::make_unique<auction::AuctionAlgorithm>(
            robot_id_, this, auction_params);
        
        robot_controller_ = std::make_unique<robot::RobotController>(
            robot_id_, this, robot_params);
        
        task_manager_ = std::make_unique<task::TaskManager>(
            robot_id_, this);
        
        // Create publishers
        status_pub_ = this->create_publisher<msg::RobotStatus>(
            "robot_status", 10);
        
        bid_pub_ = this->create_publisher<msg::Bid>(
            "bids", 10);
        
        // Create subscribers
        task_sub_ = this->create_subscription<msg::TaskArray>(
            "tasks", 10, std::bind(&RobotNode::taskCallback, this, std::placeholders::_1));
        
        bid_sub_ = this->create_subscription<msg::Bid>(
            "bids", 10, std::bind(&RobotNode::bidCallback, this, std::placeholders::_1));
        
        robot_status_sub_ = this->create_subscription<msg::RobotStatus>(
            "robot_status", 10, std::bind(&RobotNode::robotStatusCallback, this, std::placeholders::_1));
        
        auction_status_sub_ = this->create_subscription<msg::AuctionStatus>(
            "auction_status", 10, std::bind(&RobotNode::auctionStatusCallback, this, std::placeholders::_1));
        
        // Create service clients
        fail_robot_client_ = this->create_client<srv::FailRobot>("fail_robot");
        
        // Create service servers
        fail_service_ = this->create_service<srv::FailRobot>(
            "fail_robot_" + std::to_string(robot_id_),
            std::bind(&RobotNode::failRobotCallback, this, 
                      std::placeholders::_1, std::placeholders::_2));
        
        // Create timers
        heartbeat_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / heartbeat_rate_),
            std::bind(&RobotNode::publishHeartbeat, this));
        
        auction_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / auction_rate_),
            std::bind(&RobotNode::runAuction, this));
        
        controller_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / controller_rate_),
            std::bind(&RobotNode::updateController, this));
        
        // Initialize components
        robot_controller_->initialize();
        
        RCLCPP_INFO(this->get_logger(), "Robot Node %d initialized", robot_id_);
    }

private:
    /**
     * @brief Callback for task messages
     * @param msg The task array message
     */
    void taskCallback(const msg::TaskArray::SharedPtr msg) {
        // Update task list
        task_manager_->updateTasks(msg->tasks);
        
        // Initialize auction algorithm if not already done
        if (!auction_initialized_ && !msg->tasks.empty()) {
            auction_algorithm_->initialize(msg->tasks);
            auction_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Auction algorithm initialized with %zu tasks",
                msg->tasks.size());
        }
    }
    
    /**
     * @brief Callback for bid messages
     * @param msg The bid message
     */
    void bidCallback(const msg::Bid::SharedPtr msg) {
        // Ignore our own bids
        if (msg->robot_id == robot_id_) {
            return;
        }
        
        // Process bid in auction algorithm
        auction_algorithm_->processBid(*msg);
    }
    
    /**
     * @brief Callback for robot status messages
     * @param msg The robot status message
     */
    void robotStatusCallback(const msg::RobotStatus::SharedPtr msg) {
        // Ignore our own status
        if (msg->robot_id == robot_id_) {
            return;
        }
        
        // Process robot status in auction algorithm
        auction_algorithm_->processRobotStatus(*msg);
    }
    
    /**
     * @brief Callback for auction status messages
     * @param msg The auction status message
     */
    void auctionStatusCallback(const msg::AuctionStatus::SharedPtr msg) {
        // Update auction status
        auction_status_ = *msg;
    }
    
    /**
     * @brief Service callback for failing this robot
     * @param request The service request
     * @param response The service response
     */
    void failRobotCallback(
        const std::shared_ptr<srv::FailRobot::Request> request,
        std::shared_ptr<srv::FailRobot::Response> response)
    {
        if (request->robot_id == robot_id_) {
            RCLCPP_WARN(this->get_logger(), "Simulating failure of robot %d", robot_id_);
            robot_controller_->setFailed(true);
            response->success = true;
            response->message = "Robot " + std::to_string(robot_id_) + " failed successfully";
        } else {
            response->success = false;
            response->message = "This service only handles failures for robot " + 
                std::to_string(robot_id_);
        }
    }
    
    /**
     * @brief Publish heartbeat message
     */
    void publishHeartbeat() {
        // Get robot status and publish
        msg::RobotStatus status = robot_controller_->getRobotStatus();
        status_pub_->publish(status);
    }
    
    /**
     * @brief Run auction algorithm iteration
     */
    void runAuction() {
        if (!auction_initialized_ || robot_controller_->hasFailed()) {
            return;
        }
        
        // Get available tasks
        std::vector<uint32_t> available_tasks = task_manager_->getAvailableTasks();
        
        // Process auction step
        bool new_assignments = auction_algorithm_->processAuctionStep(available_tasks);
        
        // If new assignments, check if we got new tasks
        if (new_assignments) {
            std::map<uint32_t, uint32_t> assignments = auction_algorithm_->getCurrentAssignments();
            std::vector<msg::Task> current_tasks = auction_algorithm_->getCurrentTasks();
            
            for (const auto& task : current_tasks) {
                // If task is assigned to us and not currently executing
                if (task.assigned_robot == robot_id_ && 
                    task.status == msg::Task::STATUS_ASSIGNED) {
                    // Assign task to robot controller
                    bool assigned = robot_controller_->assignTask(task);
                    
                    if (assigned) {
                        RCLCPP_INFO(this->get_logger(), "Robot %d assigned task %d",
                            robot_id_, task.id);
                    }
                }
            }
        }
        
        // Publish bids for debugging
        for (const auto& bid : latest_bids_) {
            bid_pub_->publish(bid);
        }
        latest_bids_.clear();
    }
    
    /**
     * @brief Update robot controller
     */
    void updateController() {
        if (robot_controller_->hasFailed()) {
            return;
        }
        
        // Update robot controller
        bool executing = robot_controller_->update();
        
        // Update task status based on execution
        if (executing) {
            // Get current task and update progress
            msg::RobotStatus status = robot_controller_->getRobotStatus();
            
            // Find the task being executed
            for (auto& task : task_manager_->getTasks()) {
                if (task.assigned_robot == robot_id_ && 
                    task.status == msg::Task::STATUS_IN_PROGRESS) {
                    // Update task progress from robot controller
                    // (This would typically come from the robot_controller's progress tracking)
                    // task.progress = ...
                    
                    // If task is complete
                    if (task.progress >= 1.0) {
                        task.status = msg::Task::STATUS_COMPLETED;
                        RCLCPP_INFO(this->get_logger(), "Robot %d completed task %d",
                            robot_id_, task.id);
                    }
                }
            }
        }
    }

    // Robot ID and rates
    uint32_t robot_id_;
    double heartbeat_rate_;
    double auction_rate_;
    double controller_rate_;
    
    // Component modules
    std::unique_ptr<auction::AuctionAlgorithm> auction_algorithm_;
    std::unique_ptr<robot::RobotController> robot_controller_;
    std::unique_ptr<task::TaskManager> task_manager_;
    
    // State
    bool auction_initialized_ = false;
    msg::AuctionStatus auction_status_;
    std::vector<msg::Bid> latest_bids_;
    
    // Publishers
    rclcpp::Publisher<msg::RobotStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<msg::Bid>::SharedPtr bid_pub_;
    
    // Subscribers
    rclcpp::Subscription<msg::TaskArray>::SharedPtr task_sub_;
    rclcpp::Subscription<msg::Bid>::SharedPtr bid_sub_;
    rclcpp::Subscription<msg::RobotStatus>::SharedPtr robot_status_sub_;
    rclcpp::Subscription<msg::AuctionStatus>::SharedPtr auction_status_sub_;
    
    // Service clients
    rclcpp::Client<srv::FailRobot>::SharedPtr fail_robot_client_;
    
    // Service servers
    rclcpp::Service<srv::FailRobot>::SharedPtr fail_service_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr auction_timer_;
    rclcpp::TimerBase::SharedPtr controller_timer_;
};

} // namespace decentralized_auction_mm

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<decentralized_auction_mm::RobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}