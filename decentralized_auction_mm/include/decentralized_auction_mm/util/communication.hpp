#ifndef DECENTRALIZED_AUCTION_MM_COMMUNICATION_HPP
#define DECENTRALIZED_AUCTION_MM_COMMUNICATION_HPP

#include <vector>
#include <map>
#include <queue>
#include <mutex>
#include <chrono>
#include <random>
#include <rclcpp/rclcpp.hpp>

#include "decentralized_auction_mm/msg/bid.hpp"
#include "decentralized_auction_mm/msg/robot_status.hpp"

namespace decentralized_auction_mm {
namespace util {

/**
 * @class Communication
 * @brief Models realistic communication constraints for the distributed system
 * 
 * This class simulates realistic communication constraints such as delay,
 * packet loss, and bandwidth limitations for the distributed auction system.
 */
class Communication {
public:
    /**
     * @brief Constructor
     * @param robot_id The ID of the robot
     * @param node The ROS node handle
     * @param params The communication parameters
     */
    Communication(
        uint32_t robot_id,
        rclcpp::Node* node,
        const std::map<std::string, double>& params
    );

    /**
     * @brief Initialize the communication module
     */
    void initialize();

    /**
     * @brief Send a bid message
     * @param bid The bid to send
     * @return True if the message was sent (not dropped due to simulated packet loss)
     */
    bool sendBid(const msg::Bid& bid);

    /**
     * @brief Send a robot status message
     * @param status The status to send
     * @return True if the message was sent (not dropped due to simulated packet loss)
     */
    bool sendRobotStatus(const msg::RobotStatus& status);

    /**
     * @brief Process a received bid message
     * @param bid The received bid
     */
    void processReceivedBid(const msg::Bid& bid);

    /**
     * @brief Process a received robot status message
     * @param status The received status
     */
    void processReceivedRobotStatus(const msg::RobotStatus& status);

    /**
     * @brief Get current packet loss probability
     * @return Current packet loss probability
     */
    double getPacketLossProbability() const;

    /**
     * @brief Get current communication delay
     * @return Current communication delay in ms
     */
    double getCommunicationDelay() const;

    /**
     * @brief Update network conditions
     * This should be called periodically to update dynamic network conditions
     */
    void updateNetworkConditions();
    
    /**
     * @brief Set packet loss probability (for simulation purposes)
     * @param probability Probability of packet loss (0.0 to 1.0)
     */
    void setPacketLossSimulation(double probability);
    
    /**
     * @brief Set communication delay (for simulation purposes)
     * @param delay_ms Delay in milliseconds
     */
    void setCommunicationDelaySimulation(double delay_ms);

private:
    /**
     * @brief Simulate packet loss
     * @return True if packet should be dropped
     */
    bool simulatePacketLoss();

    /**
     * @brief Simulate communication delay
     * @return Delay in milliseconds
     */
    double simulateCommunicationDelay();

    /**
     * @brief Process message queue
     * Processes delayed messages that have reached their delivery time
     */
    void processMessageQueue();

    // Robot ID
    uint32_t robot_id_;

    // ROS node handle
    rclcpp::Node* node_;

    // Communication parameters
    std::map<std::string, double> params_;
    double packet_loss_probability_;
    double comm_delay_ms_;
    double bandwidth_limit_kb_s_;
    double heartbeat_frequency_hz_;

    // Message queue for delayed messages
    struct DelayedMessage {
        enum class Type { BID, ROBOT_STATUS };
        Type type;
        std::chrono::system_clock::time_point delivery_time;
        msg::Bid bid;
        msg::RobotStatus status;
    };

    std::priority_queue<
        DelayedMessage,
        std::vector<DelayedMessage>,
        std::function<bool(const DelayedMessage&, const DelayedMessage&)>
    > message_queue_;

    // Network statistics
    double current_bandwidth_usage_kb_s_;
    uint64_t messages_sent_;
    uint64_t messages_dropped_;

    // ROS publishers and subscribers
    rclcpp::Publisher<msg::Bid>::SharedPtr bid_pub_;
    rclcpp::Publisher<msg::RobotStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<msg::Bid>::SharedPtr bid_sub_;
    rclcpp::Subscription<msg::RobotStatus>::SharedPtr status_sub_;

    // Timer for processing delayed messages
    rclcpp::TimerBase::SharedPtr process_queue_timer_;

    // Random number generators
    std::mt19937 rng_;
    std::uniform_real_distribution<double> uniform_dist_;
    std::lognormal_distribution<double> delay_dist_;

    // Thread safety
    mutable std::mutex mutex_;
    
    // Logging
    rclcpp::Logger logger_;
};

} // namespace util
} // namespace decentralized_auction_mm

#endif // DECENTRALIZED_AUCTION_MM_COMMUNICATION_HPP