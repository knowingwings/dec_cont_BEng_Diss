#include "decentralized_auction_mm/task/task_manager.hpp"
#include <algorithm>
#include <set>

namespace decentralized_auction_mm {
namespace task {

TaskManager::TaskManager(
    uint32_t robot_id,
    rclcpp::Node* node)
    : robot_id_(robot_id),
      node_(node),
      cache_valid_(false),
      logger_(node->get_logger())
{
    // Initialize dependency manager
    dependency_manager_ = std::make_unique<DependencyManager>();
}

void TaskManager::updateTasks(const std::vector<msg::Task>& tasks) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Update task list
    tasks_ = tasks;
    
    // Update dependency graph
    dependency_manager_->updateGraph(tasks);
    
    // Invalidate available tasks cache
    cache_valid_ = false;
    
    RCLCPP_DEBUG(logger_, "Task Manager updated with %zu tasks", tasks.size());
}

std::vector<uint32_t> TaskManager::getAvailableTasks() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // If cache is valid, return cached result
    if (cache_valid_) {
        return available_tasks_cache_;
    }
    
    // Find completed tasks
    std::set<uint32_t> completed_tasks;
    for (const auto& task : tasks_) {
        if (task.status == msg::Task::STATUS_COMPLETED) {
            completed_tasks.insert(task.id);
        }
    }
    
    // Get available tasks from dependency manager
    available_tasks_cache_ = dependency_manager_->findAvailableTasks(completed_tasks);
    
    // Mark cache as valid
    cache_valid_ = true;
    
    return available_tasks_cache_;
}

std::vector<msg::Task> TaskManager::getTasks() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return tasks_;
}

bool TaskManager::updateTaskStatus(uint32_t task_id, uint8_t status, double progress) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Find task
    for (auto& task : tasks_) {
        if (task.id == task_id) {
            // Update status
            task.status = status;
            task.progress = progress;
            
            // Invalidate cache
            cache_valid_ = false;
            
            return true;
        }
    }
    
    // Task not found
    return false;
}

msg::Task TaskManager::getTask(uint32_t task_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Find task
    for (const auto& task : tasks_) {
        if (task.id == task_id) {
            return task;
        }
    }
    
    // Not found, return empty task
    msg::Task empty_task;
    empty_task.id = 0;
    empty_task.status = msg::Task::STATUS_PENDING;
    
    return empty_task;
}

bool TaskManager::isTaskAvailable(uint32_t task_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Get available tasks
    std::vector<uint32_t> available = getAvailableTasks();
    
    // Check if task is in available list
    return std::find(available.begin(), available.end(), task_id) != available.end();
}

double TaskManager::calculateTaskCriticality(uint32_t task_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    return dependency_manager_->calculateTaskCriticality(task_id);
}

void TaskManager::updateDependencyGraph() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Update dependency graph
    dependency_manager_->updateGraph(tasks_);
    
    // Invalidate cache
    cache_valid_ = false;
}

std::vector<uint32_t> TaskManager::findRootTasks() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<uint32_t> root_tasks;
    
    // Root tasks have no prerequisites
    for (const auto& task : tasks_) {
        if (task.prerequisites.empty()) {
            root_tasks.push_back(task.id);
        }
    }
    
    return root_tasks;
}

std::vector<uint32_t> TaskManager::findCriticalPath() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Create map of task execution times
    std::map<uint32_t, double> execution_times;
    for (const auto& task : tasks_) {
        execution_times[task.id] = task.execution_time;
    }
    
    // Get critical path from dependency manager
    return dependency_manager_->computeCriticalPath(execution_times);
}

} // namespace task
} // namespace decentralized_auction_mm