#include "decentralized_auction_mm/util/communication.hpp"
#include <functional>

namespace decentralized_auction_mm {
namespace util {

// Constructor
Communication::Communication(
    uint32_t robot_id,
    rclcpp::Node* node,
    const std::map<std::string, double>& params)
    : robot_id_(robot_id),
      node_(node),
      params_(params),
      packet_loss_probability_(0.0),
      comm_delay_ms_(0.0),
      bandwidth_limit_kb_s_(0.0),
      heartbeat_frequency_hz_(1.0),
      current_bandwidth_usage_kb_s_(0.0),
      messages_sent_(0),
      messages_dropped_(0),
      // Initialize message queue with custom comparator for priority queue
      message_queue_([](const DelayedMessage& a, const DelayedMessage& b) {
          return a.delivery_time > b.delivery_time;
      }),
      uniform_dist_(0.0, 1.0),
      logger_(node->get_logger())
{
    // Get parameters
    if (params.find("comm_delay") != params.end()) {
        comm_delay_ms_ = params.at("comm_delay");
    }
    
    if (params.find("packet_loss_prob") != params.end()) {
        packet_loss_probability_ = params.at("packet_loss_prob");
    }
    
    if (params.find("bandwidth_limit") != params.end()) {
        bandwidth_limit_kb_s_ = params.at("bandwidth_limit");
    }
    
    if (params.find("heartbeat_rate") != params.end()) {
        heartbeat_frequency_hz_ = params.at("heartbeat_rate");
    }
    
    // Initialize distribution for simulating communication delay
    // Using log-normal distribution to model realistic network delays
    double mean = comm_delay_ms_;
    double stddev = mean / 2.0;
    double mu = std::log(mean) - 0.5 * std::log(1.0 + (stddev * stddev) / (mean * mean));
    double sigma = std::sqrt(std::log(1.0 + (stddev * stddev) / (mean * mean)));
    delay_dist_ = std::lognormal_distribution<double>(mu, sigma);
    
    // Initialize RNG
    std::random_device rd;
    rng_.seed(rd());
    
    RCLCPP_INFO(logger_, "Communication module initialized for robot %d", robot_id);
    RCLCPP_INFO(logger_, "Packet loss probability: %.2f", packet_loss_probability_);
    RCLCPP_INFO(logger_, "Communication delay: %.2f ms", comm_delay_ms_);
}

void Communication::initialize() {
    // Create publishers
    bid_pub_ = node_->create_publisher<msg::Bid>("bids", 10);
    status_pub_ = node_->create_publisher<msg::RobotStatus>("robot_status", 10);
    
    // Create subscribers
    bid_sub_ = node_->create_subscription<msg::Bid>(
        "bids", 10, std::bind(&Communication::processReceivedBid, this, std::placeholders::_1));
    
    status_sub_ = node_->create_subscription<msg::RobotStatus>(
        "robot_status", 10, std::bind(&Communication::processReceivedRobotStatus, this, std::placeholders::_1));
    
    // Create timer for processing message queue
    process_queue_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&Communication::processMessageQueue, this));
}

bool Communication::sendBid(const msg::Bid& bid) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Check if message should be dropped due to packet loss
    if (simulatePacketLoss()) {
        messages_dropped_++;
        return false;
    }
    
    // Simulate communication delay
    double delay = simulateCommunicationDelay();
    
    if (delay <= 0.0) {
        // No delay, send immediately
        bid_pub_->publish(bid);
    } else {
        // Add to delayed message queue
        DelayedMessage delayed_msg;
        delayed_msg.type = DelayedMessage::Type::BID;
        delayed_msg.delivery_time = std::chrono::system_clock::now() +
                                    std::chrono::milliseconds(static_cast<int>(delay));
        delayed_msg.bid = bid;
        
        message_queue_.push(delayed_msg);
    }
    
    messages_sent_++;
    return true;
}

bool Communication::sendRobotStatus(const msg::RobotStatus& status) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Check if message should be dropped due to packet loss
    if (simulatePacketLoss()) {
        messages_dropped_++;
        return false;
    }
    
    // Simulate communication delay
    double delay = simulateCommunicationDelay();
    
    if (delay <= 0.0) {
        // No delay, send immediately
        status_pub_->publish(status);
    } else {
        // Add to delayed message queue
        DelayedMessage delayed_msg;
        delayed_msg.type = DelayedMessage::Type::ROBOT_STATUS;
        delayed_msg.delivery_time = std::chrono::system_clock::now() +
                                    std::chrono::milliseconds(static_cast<int>(delay));
        delayed_msg.status = status;
        
        message_queue_.push(delayed_msg);
    }
    
    messages_sent_++;
    return true;
}

void Communication::processReceivedBid(const msg::Bid& bid) {
    // Skip our own messages
    if (bid.robot_id == robot_id_) {
        return;
    }
    
    // Process bid (in a real implementation, this would forward to the auction algorithm)
    // This is handled by the robot node
}

void Communication::processReceivedRobotStatus(const msg::RobotStatus& status) {
    // Skip our own messages
    if (status.robot_id == robot_id_) {
        return;
    }
    
    // Process status (in a real implementation, this would forward to the auction algorithm)
    // This is handled by the robot node
}

double Communication::getPacketLossProbability() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return packet_loss_probability_;
}

double Communication::getCommunicationDelay() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return comm_delay_ms_;
}

void Communication::updateNetworkConditions() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // In a real implementation, this could update network conditions
    // based on observed performance, network load, etc.
    
    // For now, just add small random variations to simulate changing network conditions
    
    // Update packet loss probability with small variations
    double variation = 0.05 * (uniform_dist_(rng_) - 0.5);
    packet_loss_probability_ = std::max(0.0, std::min(1.0, packet_loss_probability_ + variation));
    
    // Update communication delay with small variations
    variation = 20.0 * (uniform_dist_(rng_) - 0.5);
    comm_delay_ms_ = std::max(0.0, comm_delay_ms_ + variation);
    
    // Update delay distribution
    double mean = comm_delay_ms_;
    double stddev = mean / 2.0;
    double mu = std::log(mean) - 0.5 * std::log(1.0 + (stddev * stddev) / (mean * mean));
    double sigma = std::sqrt(std::log(1.0 + (stddev * stddev) / (mean * mean)));
    delay_dist_ = std::lognormal_distribution<double>(mu, sigma);
}

void Communication::setPacketLossSimulation(double probability) {
    std::lock_guard<std::mutex> lock(mutex_);
    packet_loss_probability_ = std::max(0.0, std::min(1.0, probability));
    RCLCPP_INFO(logger_, "Setting packet loss probability to %.2f", packet_loss_probability_);
}

void Communication::setCommunicationDelaySimulation(double delay_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    comm_delay_ms_ = std::max(0.0, delay_ms);
    RCLCPP_INFO(logger_, "Setting communication delay to %.2f ms", comm_delay_ms_);
    
    // Update delay distribution
    double mean = comm_delay_ms_;
    double stddev = mean / 2.0;
    double mu = std::log(mean) - 0.5 * std::log(1.0 + (stddev * stddev) / (mean * mean));
    double sigma = std::sqrt(std::log(1.0 + (stddev * stddev) / (mean * mean)));
    delay_dist_ = std::lognormal_distribution<double>(mu, sigma);
}

bool Communication::simulatePacketLoss() {
    if (packet_loss_probability_ <= 0.0) {
        return false;
    }
    
    return uniform_dist_(rng_) < packet_loss_probability_;
}

double Communication::simulateCommunicationDelay() {
    if (comm_delay_ms_ <= 0.0) {
        return 0.0;
    }
    
    return delay_dist_(rng_);
}

void Communication::processMessageQueue() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto now = std::chrono::system_clock::now();
    
    // Process all messages that have reached their delivery time
    while (!message_queue_.empty() && message_queue_.top().delivery_time <= now) {
        const DelayedMessage& msg = message_queue_.top();
        
        if (msg.type == DelayedMessage::Type::BID) {
            bid_pub_->publish(msg.bid);
        } else if (msg.type == DelayedMessage::Type::ROBOT_STATUS) {
            status_pub_->publish(msg.status);
        }
        
        message_queue_.pop();
    }
}

} // namespace util
} // namespace decentralized_auction_mm