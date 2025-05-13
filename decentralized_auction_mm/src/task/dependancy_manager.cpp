#include "decentralized_auction_mm/task/dependency_manager.hpp"
#include <algorithm>
#include <queue>
#include <limits>

namespace decentralized_auction_mm {
namespace task {

DependencyManager::DependencyManager()
    : critical_path_valid_(false)
{
    // Initialize with empty maps
}

void DependencyManager::updateGraph(const std::vector<msg::Task>& tasks) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Clear existing dependencies
    prerequisites_.clear();
    dependents_.clear();
    task_ids_.clear();
    
    // Add all tasks to the graph
    for (const auto& task : tasks) {
        task_ids_.insert(task.id);
        prerequisites_[task.id] = std::vector<uint32_t>();
        dependents_[task.id] = std::vector<uint32_t>();
    }
    
    // Add dependencies
    for (const auto& task : tasks) {
        for (uint32_t prereq_id : task.prerequisites) {
            // Add to prerequisites map
            if (prerequisites_.find(task.id) != prerequisites_.end()) {
                prerequisites_[task.id].push_back(prereq_id);
            }
            
            // Add to dependents map
            if (dependents_.find(prereq_id) != dependents_.end()) {
                dependents_[prereq_id].push_back(task.id);
            }
        }
    }
    
    // Invalidate critical path
    critical_path_valid_ = false;
}

std::vector<uint32_t> DependencyManager::findAvailableTasks(const std::set<uint32_t>& completed_tasks) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<uint32_t> available_tasks;
    
    // Check each task
    for (uint32_t task_id : task_ids_) {
        // Skip completed tasks
        if (completed_tasks.find(task_id) != completed_tasks.end()) {
            continue;
        }
        
        // Check if all prerequisites are completed
        bool all_prereqs_completed = true;
        for (uint32_t prereq_id : prerequisites_.at(task_id)) {
            if (completed_tasks.find(prereq_id) == completed_tasks.end()) {
                all_prereqs_completed = false;
                break;
            }
        }
        
        if (all_prereqs_completed) {
            available_tasks.push_back(task_id);
        }
    }
    
    return available_tasks;
}

double DependencyManager::calculateTaskCriticality(uint32_t task_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Basic criticality is the number of tasks that depend on this one, directly or indirectly
    double criticality = 0.0;
    
    // Direct dependencies (tasks that directly depend on this one)
    if (dependents_.find(task_id) != dependents_.end()) {
        criticality += dependents_.at(task_id).size();
    }
    
    // Indirect dependencies (computed recursively)
    std::set<uint32_t> visited;
    std::queue<uint32_t> queue;
    
    // Start with direct dependents
    if (dependents_.find(task_id) != dependents_.end()) {
        for (uint32_t dependent : dependents_.at(task_id)) {
            queue.push(dependent);
            visited.insert(dependent);
        }
    }
    
    // BFS to find all indirect dependents
    while (!queue.empty()) {
        uint32_t current = queue.front();
        queue.pop();
        
        // Add weight for indirect dependents (half the value of direct dependents)
        criticality += 0.5;
        
        // Add children to queue
        if (dependents_.find(current) != dependents_.end()) {
            for (uint32_t child : dependents_.at(current)) {
                if (visited.find(child) == visited.end()) {
                    queue.push(child);
                    visited.insert(child);
                }
            }
        }
    }
    
    return criticality;
}

int DependencyManager::calculateDependencyDepth(uint32_t task_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // If no prerequisites, depth is 0
    if (prerequisites_.find(task_id) == prerequisites_.end() || 
        prerequisites_.at(task_id).empty()) {
        return 0;
    }
    
    // Calculate depth recursively
    int max_depth = 0;
    for (uint32_t prereq_id : prerequisites_.at(task_id)) {
        int prereq_depth = calculateDependencyDepth(prereq_id);
        max_depth = std::max(max_depth, prereq_depth);
    }
    
    return max_depth + 1;
}

bool DependencyManager::isOnCriticalPath(uint32_t task_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // If critical path isn't valid, we need to recompute it
    if (!critical_path_valid_) {
        // We don't have execution times, so use a default map
        std::map<uint32_t, double> default_times;
        for (uint32_t id : task_ids_) {
            default_times[id] = 1.0;
        }
        
        critical_path_cache_ = computeCriticalPath(default_times);
        critical_path_valid_ = true;
    }
    
    // Check if task is on critical path
    return std::find(critical_path_cache_.begin(), critical_path_cache_.end(), task_id) != critical_path_cache_.end();
}

std::vector<uint32_t> DependencyManager::computeCriticalPath(
    const std::map<uint32_t, double>& task_execution_times) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Memoization table for longest paths
    std::map<uint32_t, double> memo;
    
    // Find terminal tasks (no dependents)
    std::vector<uint32_t> terminal_tasks;
    for (uint32_t task_id : task_ids_) {
        if (dependents_.find(task_id) == dependents_.end() || 
            dependents_.at(task_id).empty()) {
            terminal_tasks.push_back(task_id);
        }
    }
    
    // If no terminal tasks, return empty path
    if (terminal_tasks.empty()) {
        return std::vector<uint32_t>();
    }
    
    // Find task with longest path
    uint32_t critical_terminal = terminal_tasks[0];
    double max_path_length = calculateLongestPath(critical_terminal, task_execution_times, memo);
    
    for (size_t i = 1; i < terminal_tasks.size(); ++i) {
        uint32_t task_id = terminal_tasks[i];
        double path_length = calculateLongestPath(task_id, task_execution_times, memo);
        
        if (path_length > max_path_length) {
            max_path_length = path_length;
            critical_terminal = task_id;
        }
    }
    
    // Reconstruct the critical path from the terminal task
    std::vector<uint32_t> critical_path;
    uint32_t current = critical_terminal;
    
    while (true) {
        critical_path.push_back(current);
        
        // If no prerequisites, we've reached the start
        if (prerequisites_.find(current) == prerequisites_.end() || 
            prerequisites_.at(current).empty()) {
            break;
        }
        
        // Find the prerequisite with the longest path
        uint32_t next = prerequisites_.at(current)[0];
        double max_prereq_length = calculateLongestPath(next, task_execution_times, memo);
        
        for (size_t i = 1; i < prerequisites_.at(current).size(); ++i) {
            uint32_t prereq_id = prerequisites_.at(current)[i];
            double prereq_length = calculateLongestPath(prereq_id, task_execution_times, memo);
            
            if (prereq_length > max_prereq_length) {
                max_prereq_length = prereq_length;
                next = prereq_id;
            }
        }
        
        current = next;
    }
    
    // Reverse the path to get from start to finish
    std::reverse(critical_path.begin(), critical_path.end());
    
    return critical_path;
}

std::vector<uint32_t> DependencyManager::determineExecutionOrder() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<uint32_t> order;
    std::set<uint32_t> visited;
    std::set<uint32_t> temp_mark;
    
    // Perform topological sort
    for (uint32_t task_id : task_ids_) {
        if (visited.find(task_id) == visited.end()) {
            if (!dfs(task_id, visited, temp_mark, order)) {
                // Cycle detected, return empty order
                return std::vector<uint32_t>();
            }
        }
    }
    
    // Reverse the order (DFS gives reverse topological sort)
    std::reverse(order.begin(), order.end());
    
    return order;
}

std::vector<uint32_t> DependencyManager::getPrerequisites(uint32_t task_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (prerequisites_.find(task_id) != prerequisites_.end()) {
        return prerequisites_.at(task_id);
    }
    
    return std::vector<uint32_t>();
}

std::vector<uint32_t> DependencyManager::getDependentTasks(uint32_t task_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (dependents_.find(task_id) != dependents_.end()) {
        return dependents_.at(task_id);
    }
    
    return std::vector<uint32_t>();
}

bool DependencyManager::dfs(uint32_t task_id, 
                         std::set<uint32_t>& visited, 
                         std::set<uint32_t>& temp_mark,
                         std::vector<uint32_t>& order) const {
    // Check for cycle
    if (temp_mark.find(task_id) != temp_mark.end()) {
        return false;  // Cycle detected
    }
    
    // Skip if already visited
    if (visited.find(task_id) != visited.end()) {
        return true;
    }
    
    // Mark temporarily
    temp_mark.insert(task_id);
    
    // Visit prerequisites
    if (prerequisites_.find(task_id) != prerequisites_.end()) {
        for (uint32_t prereq_id : prerequisites_.at(task_id)) {
            if (!dfs(prereq_id, visited, temp_mark, order)) {
                return false;  // Cycle detected
            }
        }
    }
    
    // Mark permanently
    visited.insert(task_id);
    temp_mark.erase(task_id);
    
    // Add to order
    order.push_back(task_id);
    
    return true;
}

double DependencyManager::calculateLongestPath(
    uint32_t task_id,
    const std::map<uint32_t, double>& execution_times,
    std::map<uint32_t, double>& memo) const {
    // Check memoization table
    if (memo.find(task_id) != memo.end()) {
        return memo[task_id];
    }
    
    // Get execution time for this task
    double task_time = 0.0;
    if (execution_times.find(task_id) != execution_times.end()) {
        task_time = execution_times.at(task_id);
    }
    
    // If no prerequisites, path length is just the task's execution time
    if (prerequisites_.find(task_id) == prerequisites_.end() || 
        prerequisites_.at(task_id).empty()) {
        memo[task_id] = task_time;
        return task_time;
    }
    
    // Calculate longest path through prerequisites
    double max_prereq_path = 0.0;
    for (uint32_t prereq_id : prerequisites_.at(task_id)) {
        double prereq_path = calculateLongestPath(prereq_id, execution_times, memo);
        max_prereq_path = std::max(max_prereq_path, prereq_path);
    }
    
    // Path length is task time plus longest prerequisite path
    double path_length = task_time + max_prereq_path;
    
    // Store in memoization table
    memo[task_id] = path_length;
    
    return path_length;
}

} // namespace task
} // namespace decentralized_auction_mm