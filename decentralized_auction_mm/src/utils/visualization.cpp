#include "decentralized_auction_mm/util/visualization.hpp"
#include <cmath>

namespace decentralized_auction_mm {
namespace util {

Visualization::Visualization(rclcpp::Node* node)
    : node_(node),
      logger_(node->get_logger())
{
}

void Visualization::initialize() {
    // Create publishers
    task_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "visualization/tasks", 10);
    
    robot_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "visualization/robots", 10);
    
    dependency_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "visualization/dependencies", 10);
    
    assignment_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "visualization/assignments", 10);
    
    bid_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "visualization/bids", 10);
    
    RCLCPP_INFO(logger_, "Visualization initialized");
}

visualization_msgs::msg::MarkerArray Visualization::createTaskMarkers(
    const std::vector<msg::Task>& tasks) {
    
    visualization_msgs::msg::MarkerArray markers;
    
    for (const auto& task : tasks) {
        // Create cube marker for task
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node_->now();
        marker.ns = "tasks";
        marker.id = task.id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = task.position;
        marker.scale = task.dimensions;
        marker.color = getTaskColor(task).to_msg();
        
        markers.markers.push_back(marker);
        
        // Create text marker for task label
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = node_->now();
        text_marker.ns = "task_labels";
        text_marker.id = task.id + 1000;  // Offset to avoid ID collision
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose = task.position;
        text_marker.pose.position.z += task.dimensions.z + 0.05;  // Position above task
        text_marker.text = "Task " + std::to_string(task.id);
        text_marker.scale.z = 0.1;  // Text size
        
        std_msgs::msg::ColorRGBA text_color;
        text_color.r = 1.0;
        text_color.g = 1.0;
        text_color.b = 1.0;
        text_color.a = 1.0;
        text_marker.color = text_color;
        
        markers.markers.push_back(text_marker);
    }
    
    return markers;
}

visualization_msgs::msg::MarkerArray Visualization::createRobotMarkers(
    const std::map<uint32_t, msg::RobotStatus>& robots) {
    
    visualization_msgs::msg::MarkerArray markers;
    
    for (const auto& [robot_id, status] : robots) {
        // Create cylinder marker for robot
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node_->now();
        marker.ns = "robots";
        marker.id = robot_id;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = status.pose;
        
        // Set size
        marker.scale.x = 0.3;  // Diameter
        marker.scale.y = 0.3;  // Diameter
        marker.scale.z = 0.2;  // Height
        
        marker.color = getRobotColor(robot_id, status.failed).to_msg();
        
        markers.markers.push_back(marker);
        
        // Create text marker for robot label
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = node_->now();
        text_marker.ns = "robot_labels";
        text_marker.id = robot_id + 2000;  // Offset to avoid ID collision
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose = status.pose;
        text_marker.pose.position.z += 0.2;  // Position above robot
        text_marker.text = "Robot " + std::to_string(robot_id);
        text_marker.scale.z = 0.1;  // Text size
        
        std_msgs::msg::ColorRGBA text_color;
        text_color.r = 1.0;
        text_color.g = 1.0;
        text_color.b = 1.0;
        text_color.a = 1.0;
        text_marker.color = text_color;
        
        markers.markers.push_back(text_marker);
    }
    
    return markers;
}

visualization_msgs::msg::MarkerArray Visualization::createDependencyMarkers(
    const std::vector<msg::Task>& tasks) {
    
    visualization_msgs::msg::MarkerArray markers;
    int marker_id = 0;
    
    // Create a map of task ID to task for quick lookup
    std::map<uint32_t, const msg::Task*> task_map;
    for (const auto& task : tasks) {
        task_map[task.id] = &task;
    }
    
    for (const auto& task : tasks) {
        for (uint32_t prereq_id : task.prerequisites) {
            // Check if prerequisite task exists
            if (task_map.find(prereq_id) != task_map.end()) {
                const msg::Task* prereq_task = task_map[prereq_id];
                
                // Create arrow marker for dependency
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = node_->now();
                marker.ns = "dependencies";
                marker.id = marker_id++;
                marker.type = visualization_msgs::msg::Marker::ARROW;
                marker.action = visualization_msgs::msg::Marker::ADD;
                
                // Set start and end points
                marker.points.push_back(prereq_task->position.position);
                marker.points.push_back(task.position.position);
                
                // Set size
                marker.scale.x = 0.02;  // Shaft diameter
                marker.scale.y = 0.04;  // Head diameter
                marker.scale.z = 0.02;  // Head length
                
                // Set color
                std_msgs::msg::ColorRGBA arrow_color;
                arrow_color.r = 0.5;
                arrow_color.g = 0.5;
                arrow_color.b = 0.5;
                arrow_color.a = 0.5;
                marker.color = arrow_color;
                
                markers.markers.push_back(marker);
            }
        }
    }
    
    return markers;
}

visualization_msgs::msg::MarkerArray Visualization::createAssignmentMarkers(
    const std::vector<msg::Task>& tasks, 
    const std::map<uint32_t, msg::RobotStatus>& robots) {
    
    visualization_msgs::msg::MarkerArray markers;
    int marker_id = 0;
    
    for (const auto& task : tasks) {
        // Skip unassigned or completed tasks
        if (task.assigned_robot == 0 || task.status == msg::Task::STATUS_COMPLETED) {
            continue;
        }
        
        // Check if assigned robot exists
        if (robots.find(task.assigned_robot) != robots.end()) {
            const msg::RobotStatus& robot_status = robots.at(task.assigned_robot);
            
            // Create arrow marker for assignment
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = node_->now();
            marker.ns = "assignments";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set start and end points
            marker.points.push_back(robot_status.pose.position);
            marker.points.push_back(task.position.position);
            
            // Set size
            marker.scale.x = 0.03;  // Shaft diameter
            marker.scale.y = 0.06;  // Head diameter
            marker.scale.z = 0.03;  // Head length
            
            // Set color based on robot ID
            marker.color = getRobotColor(task.assigned_robot, false).to_msg();
            
            markers.markers.push_back(marker);
        }
    }
    
    return markers;
}

visualization_msgs::msg::MarkerArray Visualization::createBidMarkers(
    const std::vector<msg::Bid>& bids,
    const std::map<uint32_t, msg::RobotStatus>& robots,
    const std::vector<msg::Task>& tasks) {
    
    visualization_msgs::msg::MarkerArray markers;
    int marker_id = 0;
    
    // Create a map of task ID to task for quick lookup
    std::map<uint32_t, const msg::Task*> task_map;
    for (const auto& task : tasks) {
        task_map[task.id] = &task;
    }
    
    for (const auto& bid : bids) {
        // Check if robot and task exist
        if (robots.find(bid.robot_id) != robots.end() && task_map.find(bid.task_id) != task_map.end()) {
            const msg::RobotStatus& robot_status = robots.at(bid.robot_id);
            const msg::Task* task = task_map.at(bid.task_id);
            
            // Create line marker for bid
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = node_->now();
            marker.ns = "bids";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set start and end points
            marker.points.push_back(robot_status.pose.position);
            marker.points.push_back(task->position.position);
            
            // Set size
            marker.scale.x = 0.01;  // Line width
            
            // Set color (green)
            std_msgs::msg::ColorRGBA line_color;
            line_color.r = 0.0;
            line_color.g = 0.8;
            line_color.b = 0.0;
            line_color.a = 0.6;
            marker.color = line_color;
            
            markers.markers.push_back(marker);
            
            // Create text marker for bid value
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = "map";
            text_marker.header.stamp = node_->now();
            text_marker.ns = "bid_values";
            text_marker.id = marker_id++;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set position (midpoint between robot and task)
            text_marker.pose.position.x = (robot_status.pose.position.x + task->position.position.x) / 2;
            text_marker.pose.position.y = (robot_status.pose.position.y + task->position.position.y) / 2;
            text_marker.pose.position.z = (robot_status.pose.position.z + task->position.position.z) / 2 + 0.1;
            
            // Set text
            text_marker.text = std::to_string(bid.bid_value).substr(0, 4);
            
            // Set size
            text_marker.scale.z = 0.08;  // Text size
            
            // Set color (white)
            std_msgs::msg::ColorRGBA text_color;
            text_color.r = 1.0;
            text_color.g = 1.0;
            text_color.b = 1.0;
            text_color.a = 1.0;
            text_marker.color = text_color;
            
            markers.markers.push_back(text_marker);
        }
    }
    
    return markers;
}

void Visualization::updateMarkers(
    const std::vector<msg::Task>& tasks,
    const std::map<uint32_t, msg::RobotStatus>& robots,
    const std::vector<msg::Bid>& bids) {
    
    // Create and publish all marker types
    task_markers_pub_->publish(createTaskMarkers(tasks));
    robot_markers_pub_->publish(createRobotMarkers(robots));
    dependency_markers_pub_->publish(createDependencyMarkers(tasks));
    assignment_markers_pub_->publish(createAssignmentMarkers(tasks, robots));
    bid_markers_pub_->publish(createBidMarkers(bids, robots, tasks));
}

std_msgs::msg::ColorRGBA Visualization::getTaskColor(const msg::Task& task) {
    std_msgs::msg::ColorRGBA color;
    
    switch (task.status) {
        case msg::Task::STATUS_PENDING:
            color.r = 0.5;
            color.g = 0.5;
            color.b = 0.5;
            color.a = 0.8;
            break;
        case msg::Task::STATUS_ASSIGNED:
            color.r = 0.0;
            color.g = 0.0;
            color.b = 1.0;
            color.a = 0.8;
            break;
        case msg::Task::STATUS_IN_PROGRESS:
            color.r = 1.0;
            color.g = 0.5;
            color.b = 0.0;
            color.a = 0.8;
            break;
        case msg::Task::STATUS_COMPLETED:
            color.r = 0.0;
            color.g = 1.0;
            color.b = 0.0;
            color.a = 0.8;
            break;
        case msg::Task::STATUS_FAILED:
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
            color.a = 0.8;
            break;
        default:
            color.r = 0.5;
            color.g = 0.5;
            color.b = 0.5;
            color.a = 0.8;
    }
    
    return color;
}

std_msgs::msg::ColorRGBA Visualization::getRobotColor(uint32_t robot_id, bool failed) {
    std_msgs::msg::ColorRGBA color;
    
    if (failed) {
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 1.0;
    } else {
        // Different color for each robot
        switch (robot_id) {
            case 1:
                color.r = 0.2;
                color.g = 0.2;
                color.b = 0.8;
                color.a = 1.0;
                break;
            case 2:
                color.r = 0.8;
                color.g = 0.2;
                color.b = 0.2;
                color.a = 1.0;
                break;
            default:
                // Default color for other robots
                color.r = 0.2;
                color.g = 0.8;
                color.b = 0.2;
                color.a = 1.0;
        }
    }
    
    return color;
}

} // namespace util
} // namespace decentralized_auction_mm