#include "decentralized_auction_mm/task/task_representation.hpp"
#include <cmath>
#include <algorithm>

namespace decentralized_auction_mm {
namespace task {

TaskRepresentation::TaskRepresentation(rclcpp::Logger logger)
    : logger_(logger)
{
}

std::map<std::string, double> TaskRepresentation::toInternal(const msg::Task& task_msg) {
    std::map<std::string, double> internal;
    
    // Basic task properties
    internal["id"] = static_cast<double>(task_msg.id);
    internal["status"] = static_cast<double>(task_msg.status);
    internal["assigned_robot"] = static_cast<double>(task_msg.assigned_robot);
    internal["execution_time"] = task_msg.execution_time;
    internal["progress"] = task_msg.progress;
    internal["is_collaborative"] = task_msg.is_collaborative ? 1.0 : 0.0;
    internal["current_price"] = task_msg.current_price;
    
    // Position
    internal["pos_x"] = task_msg.position.position.x;
    internal["pos_y"] = task_msg.position.position.y;
    internal["pos_z"] = task_msg.position.position.z;
    
    // Orientation
    internal["orient_x"] = task_msg.position.orientation.x;
    internal["orient_y"] = task_msg.position.orientation.y;
    internal["orient_z"] = task_msg.position.orientation.z;
    internal["orient_w"] = task_msg.position.orientation.w;
    
    // Dimensions
    internal["dim_x"] = task_msg.dimensions.x;
    internal["dim_y"] = task_msg.dimensions.y;
    internal["dim_z"] = task_msg.dimensions.z;
    
    // Capabilities
    for (size_t i = 0; i < task_msg.capabilities_required.size(); ++i) {
        internal["cap_" + std::to_string(i)] = task_msg.capabilities_required[i];
    }
    
    // Prerequisites
    for (size_t i = 0; i < task_msg.prerequisites.size(); ++i) {
        internal["prereq_" + std::to_string(i)] = static_cast<double>(task_msg.prerequisites[i]);
    }
    
    return internal;
}

msg::Task TaskRepresentation::toMessage(const std::map<std::string, double>& internal) {
    msg::Task task_msg;
    
    // Basic task properties
    task_msg.id = static_cast<uint32_t>(internal.at("id"));
    task_msg.name = "Task_" + std::to_string(task_msg.id);
    task_msg.status = static_cast<uint8_t>(internal.at("status"));
    task_msg.assigned_robot = static_cast<int32_t>(internal.at("assigned_robot"));
    task_msg.execution_time = internal.at("execution_time");
    task_msg.progress = internal.at("progress");
    task_msg.is_collaborative = (internal.at("is_collaborative") > 0.5);
    task_msg.current_price = internal.at("current_price");
    
    // Position
    task_msg.position.position.x = internal.at("pos_x");
    task_msg.position.position.y = internal.at("pos_y");
    task_msg.position.position.z = internal.at("pos_z");
    
    // Orientation
    task_msg.position.orientation.x = internal.at("orient_x");
    task_msg.position.orientation.y = internal.at("orient_y");
    task_msg.position.orientation.z = internal.at("orient_z");
    task_msg.position.orientation.w = internal.at("orient_w");
    
    // Dimensions
    task_msg.dimensions.x = internal.at("dim_x");
    task_msg.dimensions.y = internal.at("dim_y");
    task_msg.dimensions.z = internal.at("dim_z");
    
    // Capabilities
    task_msg.capabilities_required.clear();
    for (size_t i = 0; i < 10; ++i) {  // Assuming max 10 capabilities
        std::string key = "cap_" + std::to_string(i);
        if (internal.find(key) != internal.end()) {
            task_msg.capabilities_required.push_back(internal.at(key));
        } else {
            break;  // No more capabilities
        }
    }
    
    // Prerequisites
    task_msg.prerequisites.clear();
    for (size_t i = 0; i < 10; ++i) {  // Assuming max 10 prerequisites
        std::string key = "prereq_" + std::to_string(i);
        if (internal.find(key) != internal.end()) {
            task_msg.prerequisites.push_back(static_cast<uint32_t>(internal.at(key)));
        } else {
            break;  // No more prerequisites
        }
    }
    
    return task_msg;
}

double TaskRepresentation::calculateDistance(const msg::Task& task1, const msg::Task& task2) {
    double dx = task1.position.position.x - task2.position.position.x;
    double dy = task1.position.position.y - task2.position.position.y;
    double dz = task1.position.position.z - task2.position.position.z;
    
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

double TaskRepresentation::calculateSimilarity(const msg::Task& task1, const msg::Task& task2) {
    // Compare capability requirements using cosine similarity
    std::vector<double> cap1(task1.capabilities_required.begin(), task1.capabilities_required.end());
    std::vector<double> cap2(task2.capabilities_required.begin(), task2.capabilities_required.end());
    
    // Ensure vectors are same length
    size_t max_len = std::max(cap1.size(), cap2.size());
    cap1.resize(max_len, 0.0);
    cap2.resize(max_len, 0.0);
    
    // Normalize vectors
    std::vector<double> norm1 = normalizeVector(cap1);
    std::vector<double> norm2 = normalizeVector(cap2);
    
    // Calculate cosine similarity
    return dotProduct(norm1, norm2);
}

bool TaskRepresentation::canExecute(const msg::Task& task, const std::vector<double>& capabilities) {
    // For collaborative tasks, always return true (requires both robots)
    if (task.is_collaborative) {
        return true;
    }
    
    // Check if robot capabilities meet task requirements
    double capability_match = 0.0;
    size_t min_size = std::min(task.capabilities_required.size(), capabilities.size());
    
    for (size_t i = 0; i < min_size; ++i) {
        capability_match += capabilities[i] * task.capabilities_required[i];
    }
    
    // Normalize by vector lengths
    double task_norm = 0.0;
    double capabilities_norm = 0.0;
    
    for (double cap : task.capabilities_required) {
        task_norm += cap * cap;
    }
    
    for (double cap : capabilities) {
        capabilities_norm += cap * cap;
    }
    
    task_norm = std::sqrt(task_norm);
    capabilities_norm = std::sqrt(capabilities_norm);
    
    if (task_norm > 0.0 && capabilities_norm > 0.0) {
        capability_match /= (task_norm * capabilities_norm);
    } else {
        capability_match = 0.0;
    }
    
    // Check if match exceeds threshold
    const double threshold = 0.7;  // 70% capability match required
    return capability_match >= threshold;
}

std::vector<double> TaskRepresentation::normalizeVector(const std::vector<double>& vec) {
    double magnitude = 0.0;
    for (double val : vec) {
        magnitude += val * val;
    }
    magnitude = std::sqrt(magnitude);
    
    std::vector<double> normalized(vec.size(), 0.0);
    if (magnitude > 1e-10) {
        for (size_t i = 0; i < vec.size(); ++i) {
            normalized[i] = vec[i] / magnitude;
        }
    }
    
    return normalized;
}

double TaskRepresentation::dotProduct(const std::vector<double>& vec1, const std::vector<double>& vec2) {
    double result = 0.0;
    size_t min_size = std::min(vec1.size(), vec2.size());
    
    for (size_t i = 0; i < min_size; ++i) {
        result += vec1[i] * vec2[i];
    }
    
    return result;
}

} // namespace task
} // namespace decentralized_auction_mm