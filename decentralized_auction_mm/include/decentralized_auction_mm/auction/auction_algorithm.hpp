#ifndef DECENTRALIZED_AUCTION_MM_AUCTION_ALGORITHM_HPP
#define DECENTRALIZED_AUCTION_MM_AUCTION_ALGORITHM_HPP

#include <vector>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include "decentralized_auction_mm/msg/task.hpp"
#include "decentralized_auction_mm/msg/task_array.hpp"
#include "decentralized_auction_mm/msg/bid.hpp"
#include "decentralized_auction_mm/msg/bid_array.hpp"
#include "decentralized_auction_mm/msg/auction_status.hpp"
#include "decentralized_auction_mm/msg/robot_status.hpp"

#include "decentralized_auction_mm/auction/bid_calculator.hpp"
#include "decentralized_auction_mm/auction/consensus_protocol.hpp"
#include "decentralized_auction_mm/auction/recovery_mechanism.hpp"

namespace decentralized_auction_mm {
namespace auction {

/**
 * @class AuctionAlgorithm
 * @brief Implements a distributed auction algorithm for task allocation
 * 
 * This class implements the core distributed auction algorithm based on
 * Zavlanos et al. (2008) with extensions for task dependencies and 
 * failure recovery as described in the dissertation.
 */
class AuctionAlgorithm {
public:
    /**
     * @brief Constructor
     * @param robot_id The ID of the robot running this auction instance
     * @param node The ROS node handle
     * @param params The auction parameters
     */
    AuctionAlgorithm(
        uint32_t robot_id, 
        rclcpp::Node* node,
        const std::map<std::string, double>& params
    );

    /**
     * @brief Initialize the auction algorithm
     * @param tasks The initial set of tasks
     */
    void initialize(const std::vector<msg::Task>& tasks);

    /**
     * @brief Process a distributed auction step
     * @param available_tasks IDs of tasks that are available for assignment
     * @return True if new assignments were made
     */
    bool processAuctionStep(const std::vector<uint32_t>& available_tasks);

    /**
     * @brief Process a bid from another robot
     * @param bid The bid to process
     */
    void processBid(const msg::Bid& bid);

    /**
     * @brief Process a robot status update
     * @param status The robot status to process
     */
    void processRobotStatus(const msg::RobotStatus& status);

    /**
     * @brief Get the current task assignments
     * @return Map of task ID to robot ID
     */
    std::map<uint32_t, uint32_t> getCurrentAssignments() const;

    /**
     * @brief Set a robot as failed
     * @param robot_id The ID of the failed robot
     */
    void setRobotFailed(uint32_t robot_id);

    /**
     * @brief Check if recovery mode is active
     * @return True if in recovery mode
     */
    bool isInRecoveryMode() const;

    /**
     * @brief Get the current workload (total assigned task time)
     * @return The current workload
     */
    double getCurrentWorkload() const;

    /**
     * @brief Get the auction status message
     * @return AuctionStatus message
     */
    msg::AuctionStatus getAuctionStatus() const;

    /**
     * @brief Get the current tasks with their prices and assignments
     * @return Vector of tasks
     */
    std::vector<msg::Task> getCurrentTasks() const;

private:
    /**
     * @brief Calculate bids for available tasks
     * @param available_tasks IDs of tasks that are available for assignment
     * @return Vector of bids
     */
    std::vector<msg::Bid> calculateBids(const std::vector<uint32_t>& available_tasks);

    /**
     * @brief Update task assignments based on bids
     * @param bids The bids to process
     * @return True if assignments changed
     */
    bool updateAssignments(const std::vector<msg::Bid>& bids);

    /**
     * @brief Update task prices based on assignments
     * @param task_id The task ID
     * @param bid_value The winning bid value
     */
    void updateTaskPrice(uint32_t task_id, double bid_value);

    /**
     * @brief Initiate recovery after robot failure
     * @param failed_robot_id The ID of the failed robot
     */
    void initiateRecovery(uint32_t failed_robot_id);

    /**
     * @brief Process recovery step for reassigning tasks
     * @param available_tasks IDs of tasks that are available for assignment
     * @return True if new assignments were made
     */
    bool processRecoveryStep(const std::vector<uint32_t>& available_tasks);

    // Robot ID
    uint32_t robot_id_;

    // ROS node handle
    rclcpp::Node* node_;

    // Auction parameters
    std::map<std::string, double> params_;

    // Auction state
    uint32_t iteration_;
    bool in_recovery_mode_;
    uint32_t recovery_iteration_;
    uint32_t failed_robot_id_;
    
    // Current tasks, prices, and assignments
    std::vector<msg::Task> tasks_;
    std::map<uint32_t, double> task_prices_;
    std::map<uint32_t, uint32_t> task_assignments_;
    std::map<uint32_t, uint32_t> initial_assignments_; // For recovery analysis
    std::map<uint32_t, double> task_progress_;
    std::map<uint32_t, uint32_t> task_oscillation_count_;
    std::map<uint32_t, uint32_t> task_last_robot_;
    std::map<uint32_t, uint32_t> unassigned_iterations_;

    // Robot status information
    std::map<uint32_t, msg::RobotStatus> robot_status_;
    std::map<uint32_t, double> robot_workloads_;
    std::map<uint32_t, bool> robot_failed_;

    // Component modules
    std::unique_ptr<BidCalculator> bid_calculator_;
    std::unique_ptr<ConsensusProtocol> consensus_protocol_;
    std::unique_ptr<RecoveryMechanism> recovery_mechanism_;

    // Thread safety
    mutable std::mutex mutex_;
    
    // Logging
    rclcpp::Logger logger_;
};

} // namespace auction
} // namespace decentralized_auction_mm

#endif // DECENTRALIZED_AUCTION_MM_AUCTION_ALGORITHM_HPP