#include <memory>
#include <string>
#include <vector>
#include <map>
#include <random>
#include <chrono>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "decentralized_auction_mm/msg/task.hpp"
#include "decentralized_auction_mm/msg/task_array.hpp"
#include "decentralized_auction_mm/msg/auction_status.hpp"

using namespace std::chrono_literals;

namespace decentralized_auction_mm {

/**
 * @class TaskGeneratorNode
 * @brief Node for generating assembly tasks
 * 
 * This node creates tasks for the auction system, distributes them in the workspace,
 * and adds dependencies between tasks to form a directed acyclic graph.
 */
class TaskGeneratorNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     */
    TaskGeneratorNode() : Node("task_generator_node") {
        // Declare and get parameters
        this->declare_parameter("num_tasks", 10);
        this->declare_parameter("workspace_size", std::vector<double>{4.0, 4.0});
        this->declare_parameter("min_task_spacing", 0.5);
        this->declare_parameter("dependency_probability", 0.3);
        this->declare_parameter("max_dependencies_per_task", 3);
        this->declare_parameter("dependency_mode", "spatial");
        this->declare_parameter("min_execution_time", 5.0);
        this->declare_parameter("max_execution_time", 15.0);
        this->declare_parameter("collaborative_task_ratio", 0.2);
        this->declare_parameter("generation_mode", "grid");
        this->declare_parameter("grid_rows", 3);
        this->declare_parameter("grid_cols", 4);
        this->declare_parameter("grid_jitter", 0.2);
        this->declare_parameter("dynamic_generation", false);
        this->declare_parameter("generation_interval", 30.0);
        this->declare_parameter("max_total_tasks", 20);
        
        num_tasks_ = this->get_parameter("num_tasks").as_int();
        workspace_size_ = this->get_parameter("workspace_size").as_double_array();
        min_task_spacing_ = this->get_parameter("min_task_spacing").as_double();
        dependency_probability_ = this->get_parameter("dependency_probability").as_double();
        max_dependencies_per_task_ = this->get_parameter("max_dependencies_per_task").as_int();
        dependency_mode_ = this->get_parameter("dependency_mode").as_string();
        min_execution_time_ = this->get_parameter("min_execution_time").as_double();
        max_execution_time_ = this->get_parameter("max_execution_time").as_double();
        collaborative_task_ratio_ = this->get_parameter("collaborative_task_ratio").as_double();
        generation_mode_ = this->get_parameter("generation_mode").as_string();
        grid_rows_ = this->get_parameter("grid_rows").as_int();
        grid_cols_ = this->get_parameter("grid_cols").as_int();
        grid_jitter_ = this->get_parameter("grid_jitter").as_double();
        dynamic_generation_ = this->get_parameter("dynamic_generation").as_bool();
        generation_interval_ = this->get_parameter("generation_interval").as_double();
        max_total_tasks_ = this->get_parameter("max_total_tasks").as_int();
        
        // Initialize capability patterns
        this->declare_parameter("capability_patterns.0", std::vector<double>{0.8, 0.4, 0.6, 0.2, 0.3});
        this->declare_parameter("capability_patterns.1", std::vector<double>{0.3, 0.9, 0.2, 0.8, 0.4});
        this->declare_parameter("capability_patterns.2", std::vector<double>{0.5, 0.5, 0.7, 0.5, 0.9});
        
        capability_patterns_.push_back(this->get_parameter("capability_patterns.0").as_double_array());
        capability_patterns_.push_back(this->get_parameter("capability_patterns.1").as_double_array());
        capability_patterns_.push_back(this->get_parameter("capability_patterns.2").as_double_array());
        
        // Create publishers
        task_pub_ = this->create_publisher<msg::TaskArray>("tasks", 10);
        task_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization/tasks", 10);
        
        // Create subscribers
        auction_status_sub_ = this->create_subscription<msg::AuctionStatus>(
            "auction_status", 10, std::bind(&TaskGeneratorNode::auctionStatusCallback, this, std::placeholders::_1));
        
        // Create timers
        if (dynamic_generation_) {
            generation_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(generation_interval_),
                std::bind(&TaskGeneratorNode::generateDynamicTasks, this));
        }
        
        // Initialize RNG
        rng_.seed(std::chrono::system_clock::now().time_since_epoch().count());
        
        // Generate initial tasks
        generateInitialTasks();
        
        RCLCPP_INFO(this->get_logger(), "Task Generator Node initialized with %d tasks", num_tasks_);
    }

private:
    /**
     * @brief Generate initial tasks
     */
    void generateInitialTasks() {
        // Generate task positions
        std::vector<geometry_msgs::msg::Point> positions;
        
        if (generation_mode_ == "grid") {
            positions = generateGridPositions();
        } else if (generation_mode_ == "clustered") {
            positions = generateClusteredPositions();
        } else {
            // Default to random generation
            positions = generateRandomPositions();
        }
        
        // Limit to requested number of tasks
        if (positions.size() > static_cast<size_t>(num_tasks_)) {
            positions.resize(num_tasks_);
        }
        
        // Create tasks with the generated positions
        for (size_t i = 0; i < positions.size(); ++i) {
            msg::Task task;
            task.header.stamp = this->now();
            task.id = i + 1;  // Start IDs from 1
            task.name = "Task_" + std::to_string(task.id);
            
            // Set position
            task.position.position = positions[i];
            task.position.orientation.w = 1.0;  // Default quaternion
            
            // Set dimensions for visualization
            task.dimensions.x = 0.1;
            task.dimensions.y = 0.1;
            task.dimensions.z = 0.1;
            
            // Set execution time (random within range)
            std::uniform_real_distribution<double> time_dist(min_execution_time_, max_execution_time_);
            task.execution_time = time_dist(rng_);
            
            // Set capability requirements
            setTaskCapabilities(task);
            
            // Determine if collaborative
            std::uniform_real_distribution<double> collab_dist(0.0, 1.0);
            task.is_collaborative = (collab_dist(rng_) < collaborative_task_ratio_);
            
            // Set initial status
            task.status = msg::Task::STATUS_PENDING;
            task.assigned_robot = 0;  // Unassigned
            task.progress = 0.0;
            task.current_price = 0.0;
            
            // Initially no prerequisites (will be set later)
            task.prerequisites.clear();
            
            tasks_.push_back(task);
        }
        
        // Add dependencies between tasks
        addTaskDependencies();
        
        // Publish tasks
        publishTasks();
    }
    
    /**
     * @brief Generate random positions for tasks
     * @return Vector of points
     */
    std::vector<geometry_msgs::msg::Point> generateRandomPositions() {
        std::vector<geometry_msgs::msg::Point> positions;
        
        // Distribution for random positions
        std::uniform_real_distribution<double> x_dist(0.5, workspace_size_[0] - 0.5);
        std::uniform_real_distribution<double> y_dist(0.5, workspace_size_[1] - 0.5);
        
        // Generate positions with minimum spacing
        for (int i = 0; i < num_tasks_; ++i) {
            bool valid_position = false;
            geometry_msgs::msg::Point point;
            
            // Try to generate a valid position (not too close to others)
            for (int attempt = 0; attempt < 100 && !valid_position; ++attempt) {
                point.x = x_dist(rng_);
                point.y = y_dist(rng_);
                point.z = 0.8;  // Height on the table
                
                valid_position = true;
                
                // Check distance to existing positions
                for (const auto& existing : positions) {
                    double dx = point.x - existing.x;
                    double dy = point.y - existing.y;
                    double distance = std::sqrt(dx*dx + dy*dy);
                    
                    if (distance < min_task_spacing_) {
                        valid_position = false;
                        break;
                    }
                }
            }
            
            if (valid_position) {
                positions.push_back(point);
            } else {
                RCLCPP_WARN(this->get_logger(), "Could not find valid position for task %d", i+1);
            }
        }
        
        return positions;
    }
    
    /**
     * @brief Generate grid-based positions for tasks
     * @return Vector of points
     */
    std::vector<geometry_msgs::msg::Point> generateGridPositions() {
        std::vector<geometry_msgs::msg::Point> positions;
        
        // Calculate grid spacing
        double x_spacing = (workspace_size_[0] - 1.0) / (grid_cols_ - 1);
        double y_spacing = (workspace_size_[1] - 1.0) / (grid_rows_ - 1);
        
        // Distribution for jitter
        std::uniform_real_distribution<double> jitter_dist(-grid_jitter_, grid_jitter_);
        
        // Generate grid positions with jitter
        for (int row = 0; row < grid_rows_; ++row) {
            for (int col = 0; col < grid_cols_; ++col) {
                geometry_msgs::msg::Point point;
                
                // Base grid position
                point.x = 0.5 + col * x_spacing;
                point.y = 0.5 + row * y_spacing;
                
                // Add jitter
                point.x += jitter_dist(rng_);
                point.y += jitter_dist(rng_);
                
                // Ensure within workspace
                point.x = std::max(0.3, std::min(workspace_size_[0] - 0.3, point.x));
                point.y = std::max(0.3, std::min(workspace_size_[1] - 0.3, point.y));
                
                // Set height
                point.z = 0.8;  // Height on the table
                
                positions.push_back(point);
            }
        }
        
        // Shuffle positions to randomize grid order
        std::shuffle(positions.begin(), positions.end(), rng_);
        
        return positions;
    }
    
    /**
     * @brief Generate clustered positions for tasks
     * @return Vector of points
     */
    std::vector<geometry_msgs::msg::Point> generateClusteredPositions() {
        std::vector<geometry_msgs::msg::Point> positions;
        
        // Create 2-3 cluster centers
        std::uniform_int_distribution<int> num_clusters_dist(2, 3);
        int num_clusters = num_clusters_dist(rng_);
        
        std::vector<geometry_msgs::msg::Point> cluster_centers;
        std::uniform_real_distribution<double> x_dist(0.7, workspace_size_[0] - 0.7);
        std::uniform_real_distribution<double> y_dist(0.7, workspace_size_[1] - 0.7);
        
        for (int i = 0; i < num_clusters; ++i) {
            geometry_msgs::msg::Point center;
            center.x = x_dist(rng_);
            center.y = y_dist(rng_);
            center.z = 0.8;
            
            cluster_centers.push_back(center);
        }
        
        // Distribute tasks around cluster centers
        std::uniform_int_distribution<int> cluster_dist(0, num_clusters - 1);
        std::normal_distribution<double> offset_dist(0.0, 0.4);
        
        for (int i = 0; i < num_tasks_; ++i) {
            int cluster_idx = cluster_dist(rng_);
            geometry_msgs::msg::Point center = cluster_centers[cluster_idx];
            
            geometry_msgs::msg::Point point;
            point.x = center.x + offset_dist(rng_);
            point.y = center.y + offset_dist(rng_);
            point.z = center.z;
            
            // Ensure within workspace
            point.x = std::max(0.3, std::min(workspace_size_[0] - 0.3, point.x));
            point.y = std::max(0.3, std::min(workspace_size_[1] - 0.3, point.y));
            
            positions.push_back(point);
        }
        
        return positions;
    }
    
    /**
     * @brief Set capability requirements for a task
     * @param task The task to modify
     */
    void setTaskCapabilities(msg::Task& task) {
        // Select a capability pattern based on task ID
        std::vector<double> base_pattern = capability_patterns_[task.id % capability_patterns_.size()];
        
        // Add small random variations
        std::normal_distribution<double> variation_dist(0.0, 0.1);
        
        std::vector<double> capabilities;
        for (double cap : base_pattern) {
            double variation = variation_dist(rng_);
            capabilities.push_back(std::max(0.1, std::min(1.0, cap + variation)));
        }
        
        task.capabilities_required = capabilities;
    }
    
    /**
     * @brief Add dependencies between tasks
     */
    void addTaskDependencies() {
        if (tasks_.size() <= 1) {
            return;  // No dependencies needed for 0 or 1 task
        }
        
        if (dependency_mode_ == "spatial") {
            addSpatialDependencies();
        } else if (dependency_mode_ == "structured") {
            addStructuredDependencies();
        } else {
            // Default to random dependencies
            addRandomDependencies();
        }
        
        // Verify we don't have cycles
        if (!verifyAcyclicDependencies()) {
            RCLCPP_WARN(this->get_logger(), "Cyclic dependencies detected, resetting dependencies");
            
            // Reset dependencies
            for (auto& task : tasks_) {
                task.prerequisites.clear();
            }
            
            // Add simplified dependencies
            addSimplifiedDependencies();
        }
    }
    
    /**
     * @brief Add random dependencies between tasks
     */
    void addRandomDependencies() {
        std::uniform_real_distribution<double> prob_dist(0.0, 1.0);
        
        // Sort tasks by ID to ensure we don't create cycles
        std::sort(tasks_.begin(), tasks_.end(), 
                 [](const msg::Task& a, const msg::Task& b) {
                     return a.id < b.id;
                 });
        
        // For each task (except the first), consider adding dependencies
        for (size_t i = 1; i < tasks_.size(); ++i) {
            // Consider all previous tasks as potential prerequisites
            for (size_t j = 0; j < i; ++j) {
                // Add dependency with specified probability
                if (prob_dist(rng_) < dependency_probability_) {
                    // Limit maximum number of dependencies
                    if (tasks_[i].prerequisites.size() < static_cast<size_t>(max_dependencies_per_task_)) {
                        tasks_[i].prerequisites.push_back(tasks_[j].id);
                    }
                }
            }
        }
    }
    
    /**
     * @brief Add dependencies based on spatial proximity
     */
    void addSpatialDependencies() {
        // Sort tasks by x-coordinate
        std::sort(tasks_.begin(), tasks_.end(), 
                 [](const msg::Task& a, const msg::Task& b) {
                     return a.position.position.x < b.position.position.x;
                 });
        
        std::uniform_real_distribution<double> prob_dist(0.0, 1.0);
        
        // For each task (except the first), consider adding dependencies to nearby tasks
        for (size_t i = 1; i < tasks_.size(); ++i) {
            // Consider nearby tasks with lower indices as potential prerequisites
            for (size_t j = 0; j < i; ++j) {
                double dx = tasks_[i].position.position.x - tasks_[j].position.position.x;
                double dy = tasks_[i].position.position.y - tasks_[j].position.position.y;
                double distance = std::sqrt(dx*dx + dy*dy);
                
                // Probability inversely proportional to distance
                double scaled_prob = dependency_probability_ * (2.0 / (1.0 + distance));
                
                // Add dependency with scaled probability
                if (prob_dist(rng_) < scaled_prob) {
                    // Limit maximum number of dependencies
                    if (tasks_[i].prerequisites.size() < static_cast<size_t>(max_dependencies_per_task_)) {
                        tasks_[i].prerequisites.push_back(tasks_[j].id);
                    }
                }
            }
        }
    }
    
    /**
     * @brief Add structured dependencies to form a meaningful DAG
     */
    void addStructuredDependencies() {
        // Number of "layers" in the dependency structure
        int num_layers = std::min(4, static_cast<int>(tasks_.size() / 2));
        
        // Divide tasks into layers
        std::vector<std::vector<msg::Task*>> layers(num_layers);
        
        int tasks_per_layer = tasks_.size() / num_layers;
        int remaining = tasks_.size() % num_layers;
        
        int task_idx = 0;
        for (int layer = 0; layer < num_layers; ++layer) {
            int layer_size = tasks_per_layer + (layer < remaining ? 1 : 0);
            
            for (int i = 0; i < layer_size && task_idx < static_cast<int>(tasks_.size()); ++i) {
                layers[layer].push_back(&tasks_[task_idx++]);
            }
        }
        
        // Add dependencies between layers
        std::uniform_real_distribution<double> prob_dist(0.0, 1.0);
        
        for (int layer = 1; layer < num_layers; ++layer) {
            for (auto* task : layers[layer]) {
                // Each task in layer > 0 should have at least one prerequisite
                // from the previous layer
                std::vector<uint32_t> potential_prereqs;
                for (auto* prev_task : layers[layer - 1]) {
                    potential_prereqs.push_back(prev_task->id);
                }
                
                // Shuffle to randomize selection
                std::shuffle(potential_prereqs.begin(), potential_prereqs.end(), rng_);
                
                // Add at least one prerequisite
                if (!potential_prereqs.empty()) {
                    task->prerequisites.push_back(potential_prereqs[0]);
                    
                    // Add more with decreasing probability
                    for (size_t i = 1; i < potential_prereqs.size() && 
                         task->prerequisites.size() < static_cast<size_t>(max_dependencies_per_task_); ++i) {
                        if (prob_dist(rng_) < dependency_probability_) {
                            task->prerequisites.push_back(potential_prereqs[i]);
                        }
                    }
                }
            }
        }
    }
    
    /**
     * @brief Add simplified dependencies (fallback if cycles detected)
     */
    void addSimplifiedDependencies() {
        // Simple chain structure
        for (size_t i = 1; i < tasks_.size(); ++i) {
            // Each task depends on the previous one
            if (i % 2 == 0) {  // Only add dependencies to every other task
                tasks_[i].prerequisites.push_back(tasks_[i-1].id);
            }
        }
    }
    
    /**
     * @brief Verify that dependencies form an acyclic graph
     * @return True if dependencies are acyclic
     */
    bool verifyAcyclicDependencies() {
        // Create adjacency list
        std::map<uint32_t, std::vector<uint32_t>> adj_list;
        for (const auto& task : tasks_) {
            adj_list[task.id] = std::vector<uint32_t>();
        }
        
        for (const auto& task : tasks_) {
            for (uint32_t prereq : task.prerequisites) {
                adj_list[prereq].push_back(task.id);
            }
        }
        
        // Check for cycles using DFS
        std::map<uint32_t, bool> visited;
        std::map<uint32_t, bool> rec_stack;
        
        for (const auto& task : tasks_) {
            visited[task.id] = false;
            rec_stack[task.id] = false;
        }
        
        for (const auto& task : tasks_) {
            if (!visited[task.id]) {
                if (isCyclicUtil(task.id, visited, rec_stack, adj_list)) {
                    return false;  // Cycle detected
                }
            }
        }
        
        return true;  // No cycles
    }
    
    /**
     * @brief Utility function for cycle detection
     * @param v Current vertex
     * @param visited Visited vertices
     * @param rec_stack Recursion stack
     * @param adj_list Adjacency list
     * @return True if cycle detected
     */
    bool isCyclicUtil(uint32_t v, 
                      std::map<uint32_t, bool>& visited,
                      std::map<uint32_t, bool>& rec_stack,
                      std::map<uint32_t, std::vector<uint32_t>>& adj_list) {
        visited[v] = true;
        rec_stack[v] = true;
        
        for (uint32_t neighbor : adj_list[v]) {
            if (!visited[neighbor]) {
                if (isCyclicUtil(neighbor, visited, rec_stack, adj_list)) {
                    return true;
                }
            } else if (rec_stack[neighbor]) {
                return true;  // Cycle detected
            }
        }
        
        rec_stack[v] = false;
        return false;
    }
    
    /**
     * @brief Generate dynamic tasks during execution
     */
    void generateDynamicTasks() {
        if (!dynamic_generation_ || tasks_.size() >= static_cast<size_t>(max_total_tasks_)) {
            return;
        }
        
        // Only generate new tasks if auction is running
        if (auction_status_.status != msg::AuctionStatus::STATUS_RUNNING) {
            return;
        }
        
        // Generate a small batch of new tasks
        std::uniform_int_distribution<int> num_dist(1, 3);
        int new_task_count = num_dist(rng_);
        
        // Limit to max_total_tasks_
        new_task_count = std::min(new_task_count, max_total_tasks_ - static_cast<int>(tasks_.size()));
        
        if (new_task_count <= 0) {
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Generating %d new dynamic tasks", new_task_count);
        
        // Get next task ID
        uint32_t next_id = 1;
        for (const auto& task : tasks_) {
            next_id = std::max(next_id, task.id + 1);
        }
        
        // Generate positions for new tasks
        std::vector<geometry_msgs::msg::Point> positions;
        
        if (generation_mode_ == "grid") {
            positions = generateGridPositions();
        } else if (generation_mode_ == "clustered") {
            positions = generateClusteredPositions();
        } else {
            positions = generateRandomPositions();
        }
        
        // Limit to required number
        if (positions.size() > static_cast<size_t>(new_task_count)) {
            positions.resize(new_task_count);
        }
        
        // Create new tasks
        std::vector<msg::Task> new_tasks;
        
        for (size_t i = 0; i < positions.size(); ++i) {
            msg::Task task;
            task.header.stamp = this->now();
            task.id = next_id++;
            task.name = "Task_" + std::to_string(task.id);
            
            // Set position
            task.position.position = positions[i];
            task.position.orientation.w = 1.0;  // Default quaternion
            
            // Set dimensions for visualization
            task.dimensions.x = 0.1;
            task.dimensions.y = 0.1;
            task.dimensions.z = 0.1;
            
            // Set execution time (random within range)
            std::uniform_real_distribution<double> time_dist(min_execution_time_, max_execution_time_);
            task.execution_time = time_dist(rng_);
            
            // Set capability requirements
            setTaskCapabilities(task);
            
            // Determine if collaborative
            std::uniform_real_distribution<double> collab_dist(0.0, 1.0);
            task.is_collaborative = (collab_dist(rng_) < collaborative_task_ratio_);
            
            // Set initial status
            task.status = msg::Task::STATUS_PENDING;
            task.assigned_robot = 0;  // Unassigned
            task.progress = 0.0;
            task.current_price = 0.0;
            
            // Add prerequisites from existing tasks
            addDynamicTaskDependencies(task);
            
            new_tasks.push_back(task);
            tasks_.push_back(task);
        }
        
        // Publish all tasks
        publishTasks();
        
        RCLCPP_INFO(this->get_logger(), "Generated %zu new tasks, total now %zu", 
                 new_tasks.size(), tasks_.size());
    }
    
    /**
     * @brief Add dependencies for a dynamically generated task
     * @param task The task to add dependencies to
     */
    void addDynamicTaskDependencies(msg::Task& task) {
        // Only consider completed or in-progress tasks as prerequisites
        std::vector<uint32_t> potential_prereqs;
        
        for (const auto& existing : tasks_) {
            if (existing.id != task.id && 
                (existing.status == msg::Task::STATUS_COMPLETED || 
                 existing.status == msg::Task::STATUS_IN_PROGRESS)) {
                potential_prereqs.push_back(existing.id);
            }
        }
        
        // Shuffle potential prerequisites
        std::shuffle(potential_prereqs.begin(), potential_prereqs.end(), rng_);
        
        // Add dependencies with specified probability
        std::uniform_real_distribution<double> prob_dist(0.0, 1.0);
        
        for (uint32_t prereq_id : potential_prereqs) {
            if (task.prerequisites.size() >= static_cast<size_t>(max_dependencies_per_task_)) {
                break;
            }
            
            if (prob_dist(rng_) < dependency_probability_) {
                task.prerequisites.push_back(prereq_id);
            }
        }
    }
    
    /**
     * @brief Callback for auction status messages
     * @param msg The auction status message
     */
    void auctionStatusCallback(const msg::AuctionStatus::SharedPtr msg) {
        auction_status_ = *msg;
    }
    
    /**
     * @brief Publish tasks
     */
    void publishTasks() {
        msg::TaskArray task_msg;
        task_msg.header.stamp = this->now();
        task_msg.tasks = tasks_;
        
        task_pub_->publish(task_msg);
        
        // Also publish visualization markers
        publishTaskVisualization();
    }
    
    /**
     * @brief Publish visualization markers for tasks
     */
    void publishTaskVisualization() {
        visualization_msgs::msg::MarkerArray markers;
        
        // For each task, create a marker
        for (size_t i = 0; i < tasks_.size(); ++i) {
            const auto& task = tasks_[i];
            
            // Create task marker
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "tasks";
            marker.id = task.id;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set position
            marker.pose = task.position;
            
            // Set size
            marker.scale.x = task.dimensions.x;
            marker.scale.y = task.dimensions.y;
            marker.scale.z = task.dimensions.z;
            
            // Set color based on task status
            if (task.status == msg::Task::STATUS_PENDING) {
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
                marker.color.a = 0.8;
            } else if (task.status == msg::Task::STATUS_ASSIGNED) {
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                marker.color.a = 0.8;
            } else if (task.status == msg::Task::STATUS_IN_PROGRESS) {
                marker.color.r = 1.0;
                marker.color.g = 0.5;
                marker.color.b = 0.0;
                marker.color.a = 0.8;
            } else if (task.status == msg::Task::STATUS_COMPLETED) {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 0.8;
            } else if (task.status == msg::Task::STATUS_FAILED) {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 0.8;
            }
            
            markers.markers.push_back(marker);
            
            // Create text marker for task label
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = "map";
            text_marker.header.stamp = this->now();
            text_marker.ns = "task_labels";
            text_marker.id = task.id;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Position text above task
            text_marker.pose = task.position;
            text_marker.pose.position.z += 0.15;
            
            // Set text content
            text_marker.text = "Task " + std::to_string(task.id);
            
            // Set text size
            text_marker.scale.z = 0.1;
            
            // Set text color (white)
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            
            markers.markers.push_back(text_marker);
            
            // Create dependency arrows
            for (uint32_t prereq_id : task.prerequisites) {
                // Find prerequisite task
                const msg::Task* prereq_task = nullptr;
                for (const auto& t : tasks_) {
                    if (t.id == prereq_id) {
                        prereq_task = &t;
                        break;
                    }
                }
                
                if (prereq_task) {
                    visualization_msgs::msg::Marker arrow_marker;
                    arrow_marker.header.frame_id = "map";
                    arrow_marker.header.stamp = this->now();
                    arrow_marker.ns = "dependencies";
                    arrow_marker.id = task.id * 1000 + prereq_id;
                    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
                    arrow_marker.action = visualization_msgs::msg::Marker::ADD;
                    
                    // Set start and end points
                    arrow_marker.points.push_back(prereq_task->position.position);
                    arrow_marker.points.push_back(task.position.position);
                    
                    // Set arrow size
                    arrow_marker.scale.x = 0.02;  // Shaft diameter
                    arrow_marker.scale.y = 0.04;  // Head diameter
                    arrow_marker.scale.z = 0.02;  // Head length
                    
                    // Set color (gray)
                    arrow_marker.color.r = 0.5;
                    arrow_marker.color.g = 0.5;
                    arrow_marker.color.b = 0.5;
                    arrow_marker.color.a = 0.5;
                    
                    markers.markers.push_back(arrow_marker);
                }
            }
        }
        
        task_vis_pub_->publish(markers);
    }

    // Parameters
    int num_tasks_;
    std::vector<double> workspace_size_;
    double min_task_spacing_;
    double dependency_probability_;
    int max_dependencies_per_task_;
    std::string dependency_mode_;
    double min_execution_time_;
    double max_execution_time_;
    double collaborative_task_ratio_;
    std::string generation_mode_;
    int grid_rows_;
    int grid_cols_;
    double grid_jitter_;
    bool dynamic_generation_;
    double generation_interval_;
    int max_total_tasks_;
    std::vector<std::vector<double>> capability_patterns_;
    
    // Task state
    std::vector<msg::Task> tasks_;
    
    // Auction state
    msg::AuctionStatus auction_status_;
    
    // Publishers
    rclcpp::Publisher<msg::TaskArray>::SharedPtr task_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr task_vis_pub_;
    
    // Subscribers
    rclcpp::Subscription<msg::AuctionStatus>::SharedPtr auction_status_sub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr generation_timer_;
    
    // Random number generator
    std::mt19937 rng_;
};

} // namespace decentralized_auction_mm

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<decentralized_auction_mm::TaskGeneratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}