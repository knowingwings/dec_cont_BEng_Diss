#ifndef DECENTRALIZED_AUCTION_MM_VISUALIZATION_HPP
#define DECENTRALIZED_AUCTION_MM_VISUALIZATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "decentralized_auction_mm/msg/task.hpp"
#include "decentralized_auction_mm/msg/robot_status.hpp"
#include "decentralized_auction_mm/msg/bid.hpp"

namespace decentralized_auction_mm {
namespace util {

/**
 * @class Visualization
 * @brief Utilities for visualization in RViz
 * 
 * This class provides utilities for visualizing tasks, robots,
 * assignments, and auction activity in RViz.
 */
class Visualization {
public:
    /**
     * @brief Constructor
     * @param node ROS node handle
     */
    Visualization(rclcpp::Node* node);

    /**
     * @brief Initialize visualization
     */
    void initialize();

    /**
     * @brief Create task markers for visualization
     * @param tasks Vector of tasks
     * @return Marker array for tasks
     */
    visualization_msgs::msg::MarkerArray createTaskMarkers(
        const std::vector<msg::Task>& tasks);

    /**
     * @brief Create robot markers for visualization
     * @param robots Map of robot status
     * @return Marker array for robots
     */
    visualization_msgs::msg::MarkerArray createRobotMarkers(
        const std::map<uint32_t, msg::RobotStatus>& robots);

    /**
     * @brief Create dependency markers for visualization
     * @param tasks Vector of tasks
     * @return Marker array for dependencies
     */
    visualization_msgs::msg::MarkerArray createDependencyMarkers(
        const std::vector<msg::Task>& tasks);

    /**
     * @brief Create assignment markers for visualization
     * @param tasks Vector of tasks
     * @param robots Map of robot status
     * @return Marker array for assignments
     */
    visualization_msgs::msg::MarkerArray createAssignmentMarkers(
        const std::vector<msg::Task>& tasks, 
        const std::map<uint32_t, msg::RobotStatus>& robots);

    /**
     * @brief Create bid markers for visualization
     * @param bids Vector of bids
     * @param robots Map of robot status
     * @param tasks Vector of tasks
     * @return Marker array for bids
     */
    visualization_msgs::msg::MarkerArray createBidMarkers(
        const std::vector<msg::Bid>& bids,
        const std::map<uint32_t, msg::RobotStatus>& robots,
        const std::vector<msg::Task>& tasks);

    /**
     * @brief Update markers
     * @param tasks Vector of tasks
     * @param robots Map of robot status
     * @param bids Vector of bids
     */
    void updateMarkers(
        const std::vector<msg::Task>& tasks,
        const std::map<uint32_t, msg::RobotStatus>& robots,
        const std::vector<msg::Bid>& bids);

private:
    /**
     * @brief Create a color for a task based on its status
     * @param task The task
     * @return Color for visualization
     */
    std_msgs::msg::ColorRGBA getTaskColor(const msg::Task& task);

    /**
     * @brief Create a color for a robot based on its ID
     * @param robot_id Robot ID
     * @param failed Whether the robot has failed
     * @return Color for visualization
     */
    std_msgs::msg::ColorRGBA getRobotColor(uint32_t robot_id, bool failed);

    // ROS node handle
    rclcpp::Node* node_;

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr task_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr robot_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dependency_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr assignment_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bid_markers_pub_;

    // Logging
    rclcpp::Logger logger_;
};

} // namespace util
} // namespace decentralized_auction_mm

#endif // DECENTRALIZED_AUCTION_MM_VISUALIZATION_HPP