#ifndef DECENTRALIZED_AUCTION_MM_BID_CALCULATOR_HPP
#define DECENTRALIZED_AUCTION_MM_BID_CALCULATOR_HPP

#include <map>
#include <vector>
#include "decentralized_auction_mm/msg/task.hpp"
#include "decentralized_auction_mm/msg/robot_status.hpp"

namespace decentralized_auction_mm {
namespace auction {

/**
 * @class BidCalculator
 * @brief Calculates bids for tasks in the distributed auction algorithm
 */
class BidCalculator {
public:
    /**
     * @brief Constructor
     * @param robot_id The ID of the robot
     * @param params The auction parameters
     */
    BidCalculator(
        uint32_t robot_id,
        const std::map<std::string, double>& params
    );

    /**
     * @brief Calculate a bid for a task
     * @param task The task to bid on
     * @param current_workload The current workload of the robot
     * @param workload_ratio Robot's workload as a ratio of the maximum workload
     * @param workload_imbalance Global workload imbalance measure
     * @param iteration Current auction iteration
     * @param unassigned_iterations How long the task has been unassigned
     * @param in_recovery_mode Whether in recovery mode after robot failure
     * @return The calculated bid value
     */
    double calculateBid(
        const msg::Task& task,
        double current_workload,
        double workload_ratio,
        double workload_imbalance,
        uint32_t iteration,
        uint32_t unassigned_iterations,
        bool in_recovery_mode
    );

    /**
     * @brief Set the current robot position
     * @param position The current position
     */
    void setRobotPosition(const geometry_msgs::msg::Point& position);

    /**
     * @brief Set the current robot capabilities
     * @param capabilities The current capabilities
     */
    void setRobotCapabilities(const std::vector<double>& capabilities);

private:
    /**
     * @brief Calculate distance factor component of the bid
     * @param task_position Task position
     * @return Distance factor (higher for closer tasks)
     */
    double calculateDistanceFactor(const geometry_msgs::msg::Point& task_position);

    /**
     * @brief Calculate configuration cost factor of the bid
     * @param task_position Task position
     * @return Configuration cost factor (lower for easier configurations)
     */
    double calculateConfigurationFactor(const geometry_msgs::msg::Point& task_position);

    /**
     * @brief Calculate capability matching score between robot and task
     * @param task_capabilities Task required capabilities
     * @return Capability match (higher for better match)
     */
    double calculateCapabilityMatch(const std::vector<double>& task_capabilities);

    /**
     * @brief Calculate workload factor component of the bid
     * @param workload Current workload
     * @param workload_ratio Workload ratio compared to other robots
     * @return Workload factor (higher for higher workload)
     */
    double calculateWorkloadFactor(double workload, double workload_ratio);

    /**
     * @brief Calculate energy consumption factor
     * @param task_position Task position
     * @return Energy factor (higher for more energy use)
     */
    double calculateEnergyFactor(const geometry_msgs::msg::Point& task_position);

    /**
     * @brief Calculate bonus for long-unassigned tasks
     * @param unassigned_iterations How long the task has been unassigned
     * @return Bonus factor
     */
    double calculateUnassignedBonus(uint32_t unassigned_iterations);

    /**
     * @brief Calculate time urgency factor
     * @param iteration Current iteration
     * @param unassigned_iterations How long the task has been unassigned
     * @return Urgency factor
     */
    double calculateTimeUrgencyFactor(uint32_t iteration, uint32_t unassigned_iterations);

    /**
     * @brief Normalize a vector to unit length
     * @param vec Vector to normalize
     * @return Normalized vector
     */
    std::vector<double> normalizeVector(const std::vector<double>& vec);

    // Robot ID
    uint32_t robot_id_;

    // Auction parameters
    std::map<std::string, double> params_;

    // Current robot state
    geometry_msgs::msg::Point position_;
    std::vector<double> capabilities_;
};

} // namespace auction
} // namespace decentralized_auction_mm

#endif // DECENTRALIZED_AUCTION_MM_BID_CALCULATOR_HPP