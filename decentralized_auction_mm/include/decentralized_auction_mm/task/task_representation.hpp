#ifndef DECENTRALIZED_AUCTION_MM_TASK_REPRESENTATION_HPP
#define DECENTRALIZED_AUCTION_MM_TASK_REPRESENTATION_HPP

#include <vector>
#include <string>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "decentralized_auction_mm/msg/task.hpp"

namespace decentralized_auction_mm {
namespace task {

/**
 * @class TaskRepresentation
 * @brief Provides utilities for representing and manipulating tasks
 * 
 * This class encapsulates the representation of assembly tasks
 * and provides utilities for task manipulation and conversion.
 */
class TaskRepresentation {
public:
    /**
     * @brief Constructor
     * @param logger ROS logger for logging messages
     */
    TaskRepresentation(rclcpp::Logger logger);

    /**
     * @brief Convert Task message to internal representation
     * @param task_msg The task message
     * @return Internal task representation
     */
    std::map<std::string, double> toInternal(const msg::Task& task_msg);

    /**
     * @brief Convert internal representation to Task message
     * @param internal Internal task representation
     * @return Task message
     */
    msg::Task toMessage(const std::map<std::string, double>& internal);

    /**
     * @brief Calculate distance between tasks
     * @param task1 First task
     * @param task2 Second task
     * @return Euclidean distance
     */
    double calculateDistance(const msg::Task& task1, const msg::Task& task2);

    /**
     * @brief Calculate similarity between tasks based on capabilities
     * @param task1 First task
     * @param task2 Second task
     * @return Similarity score (0.0 to 1.0)
     */
    double calculateSimilarity(const msg::Task& task1, const msg::Task& task2);

    /**
     * @brief Check if a task can be executed by a robot with given capabilities
     * @param task The task
     * @param capabilities Robot capabilities
     * @return True if the robot can execute the task
     */
    bool canExecute(const msg::Task& task, const std::vector<double>& capabilities);

private:
    /**
     * @brief Normalize a vector to unit length
     * @param vec Vector to normalize
     * @return Normalized vector
     */
    std::vector<double> normalizeVector(const std::vector<double>& vec);

    /**
     * @brief Calculate dot product of two vectors
     * @param vec1 First vector
     * @param vec2 Second vector
     * @return Dot product
     */
    double dotProduct(const std::vector<double>& vec1, const std::vector<double>& vec2);

    // Logging
    rclcpp::Logger logger_;
};

} // namespace task
} // namespace decentralized_auction_mm

#endif // DECENTRALIZED_AUCTION_MM_TASK_REPRESENTATION_HPP