#ifndef DECENTRALIZED_AUCTION_MM_DEPENDENCY_MANAGER_HPP
#define DECENTRALIZED_AUCTION_MM_DEPENDENCY_MANAGER_HPP

#include <vector>
#include <map>
#include <set>
#include <mutex>
#include "decentralized_auction_mm/msg/task.hpp"

namespace decentralized_auction_mm {
namespace task {

/**
 * @class DependencyManager
 * @brief Manages task dependencies
 * 
 * This class maintains the task dependency graph and provides
 * operations for determining task ordering and criticality.
 */
class DependencyManager {
public:
    /**
     * @brief Constructor
     */
    DependencyManager();

    /**
     * @brief Update the dependency graph
     * @param tasks The tasks with prerequisites
     */
    void updateGraph(const std::vector<msg::Task>& tasks);

    /**
     * @brief Find available tasks (all prerequisites met)
     * @param completed_tasks Set of completed task IDs
     * @return Vector of available task IDs
     */
    std::vector<uint32_t> findAvailableTasks(const std::set<uint32_t>& completed_tasks) const;

    /**
     * @brief Calculate task criticality
     * @param task_id The ID of the task
     * @return Criticality score (based on dependent tasks)
     */
    double calculateTaskCriticality(uint32_t task_id) const;

    /**
     * @brief Calculate dependency depth
     * @param task_id The ID of the task
     * @return Depth in the dependency graph
     */
    int calculateDependencyDepth(uint32_t task_id) const;

    /**
     * @brief Check if a task is on the critical path
     * @param task_id The ID of the task
     * @return True if the task is on the critical path
     */
    bool isOnCriticalPath(uint32_t task_id) const;

    /**
     * @brief Compute the critical path
     * @param task_execution_times Map of task ID to execution time
     * @return Vector of task IDs on the critical path
     */
    std::vector<uint32_t> computeCriticalPath(
        const std::map<uint32_t, double>& task_execution_times) const;

    /**
     * @brief Determine execution order
     * @return Vector of task IDs in topological order
     */
    std::vector<uint32_t> determineExecutionOrder() const;

    /**
     * @brief Get direct prerequisites of a task
     * @param task_id The ID of the task
     * @return Vector of prerequisite task IDs
     */
    std::vector<uint32_t> getPrerequisites(uint32_t task_id) const;

    /**
     * @brief Get tasks that depend on a task
     * @param task_id The ID of the task
     * @return Vector of dependent task IDs
     */
    std::vector<uint32_t> getDependentTasks(uint32_t task_id) const;

private:
    /**
     * @brief Perform depth-first search for topological sorting
     * @param task_id Current task ID
     * @param visited Set of visited tasks
     * @param temp_mark Set of temporarily marked tasks
     * @param order Result of topological sort
     * @return True if successful (no cycles)
     */
    bool dfs(uint32_t task_id, 
             std::set<uint32_t>& visited, 
             std::set<uint32_t>& temp_mark,
             std::vector<uint32_t>& order) const;

    /**
     * @brief Calculate longest path to a task
     * @param task_id The ID of the task
     * @param execution_times Map of task ID to execution time
     * @param memo Memoization table
     * @return Length of longest path
     */
    double calculateLongestPath(
        uint32_t task_id,
        const std::map<uint32_t, double>& execution_times,
        std::map<uint32_t, double>& memo) const;

    // Adjacency lists for the directed graph
    std::map<uint32_t, std::vector<uint32_t>> prerequisites_;
    std::map<uint32_t, std::vector<uint32_t>> dependents_;
    
    // Task set (all task IDs)
    std::set<uint32_t> task_ids_;
    
    // Critical path cache
    mutable std::vector<uint32_t> critical_path_cache_;
    mutable bool critical_path_valid_;
    
    // Thread safety
    mutable std::mutex mutex_;
};

} // namespace task
} // namespace decentralized_auction_mm

#endif // DECENTRALIZED_AUCTION_MM_DEPENDENCY_MANAGER_HPP