#ifndef DECENTRALIZED_AUCTION_MM_CONSENSUS_PROTOCOL_HPP
#define DECENTRALIZED_AUCTION_MM_CONSENSUS_PROTOCOL_HPP

#include <map>
#include <vector>
#include <mutex>
#include <Eigen/Dense>
#include "decentralized_auction_mm/msg/robot_status.hpp"
#include "decentralized_auction_mm/msg/task.hpp"

namespace decentralized_auction_mm {
namespace auction {

/**
 * @class ConsensusProtocol
 * @brief Implements time-weighted consensus for distributed coordination
 * 
 * This class implements the time-weighted consensus protocol described
 * in the dissertation, used for maintaining consistent state across robots.
 */
class ConsensusProtocol {
public:
    /**
     * @brief Constructor
     * @param robot_id The ID of the robot
     * @param params The consensus parameters
     */
    ConsensusProtocol(
        uint32_t robot_id,
        const std::map<std::string, double>& params
    );

    /**
     * @brief Process a robot status update
     * @param status The robot status to process
     */
    void processRobotStatus(const msg::RobotStatus& status);

    /**
     * @brief Perform a consensus update on task assignments
     * @param local_assignments Local task assignments (task_id -> robot_id)
     * @return Updated task assignments after consensus
     */
    std::map<uint32_t, uint32_t> updateTaskAssignments(
        const std::map<uint32_t, uint32_t>& local_assignments);

    /**
     * @brief Perform a consensus update on task prices
     * @param local_prices Local task prices (task_id -> price)
     * @return Updated task prices after consensus
     */
    std::map<uint32_t, double> updateTaskPrices(
        const std::map<uint32_t, double>& local_prices);

    /**
     * @brief Get the exponential convergence rate
     * @return Theoretical convergence rate
     */
    double getConvergenceRate() const;

    /**
     * @brief Get the estimated consensus error
     * @return Current consensus error
     */
    double getConsensusError() const;

private:
    /**
     * @brief Calculate time-weighted factor
     * @param last_update_time Time since last update
     * @return Weight based on time decay
     */
    double calculateTimeWeight(uint64_t last_update_time);

    /**
     * @brief Create information state vector
     * @param assignments Task assignments
     * @param prices Task prices
     * @return State vector
     */
    Eigen::VectorXd createStateVector(
        const std::map<uint32_t, uint32_t>& assignments,
        const std::map<uint32_t, double>& prices);

    /**
     * @brief Extract assignments and prices from state vector
     * @param state State vector
     * @param task_ids Vector of task IDs
     * @param assignments Output task assignments
     * @param prices Output task prices
     */
    void extractStateComponents(
        const Eigen::VectorXd& state,
        const std::vector<uint32_t>& task_ids,
        std::map<uint32_t, uint32_t>& assignments,
        std::map<uint32_t, double>& prices);

    // Robot ID
    uint32_t robot_id_;

    // Consensus parameters
    std::map<std::string, double> params_;
    double gamma_; // Base weight factor
    double lambda_; // Information decay rate

    // Robot status information
    std::map<uint32_t, msg::RobotStatus> robot_status_;
    std::map<uint32_t, uint64_t> last_update_times_;
    std::map<uint32_t, Eigen::VectorXd> robot_states_;

    // Task information
    std::vector<uint32_t> task_ids_;

    // Consensus metrics
    double consensus_error_;

    // Thread safety
    mutable std::mutex mutex_;
};

} // namespace auction
} // namespace decentralized_auction_mm

#endif // DECENTRALIZED_AUCTION_MM_CONSENSUS_PROTOCOL_HPP