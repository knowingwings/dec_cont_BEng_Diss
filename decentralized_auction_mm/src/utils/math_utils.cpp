#include "decentralized_auction_mm/util/math_utils.hpp"
#include <algorithm>

namespace decentralized_auction_mm {
namespace util {

double MathUtils::euclideanDistance(const geometry_msgs::msg::Point& p1, 
                                   const geometry_msgs::msg::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

double MathUtils::manhattanDistance(const geometry_msgs::msg::Point& p1, 
                                   const geometry_msgs::msg::Point& p2) {
    return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y) + std::abs(p1.z - p2.z);
}

double MathUtils::cosineSimilarity(const std::vector<double>& v1, 
                                  const std::vector<double>& v2) {
    if (v1.empty() || v2.empty()) {
        return 0.0;
    }
    
    // Calculate dot product
    double dot_product = 0.0;
    size_t min_size = std::min(v1.size(), v2.size());
    
    for (size_t i = 0; i < min_size; ++i) {
        dot_product += v1[i] * v2[i];
    }
    
    // Calculate magnitudes
    double mag1 = 0.0;
    double mag2 = 0.0;
    
    for (double val : v1) {
        mag1 += val * val;
    }
    
    for (double val : v2) {
        mag2 += val * val;
    }
    
    mag1 = std::sqrt(mag1);
    mag2 = std::sqrt(mag2);
    
    // Calculate cosine similarity
    if (mag1 > 0.0 && mag2 > 0.0) {
        return dot_product / (mag1 * mag2);
    } else {
        return 0.0;
    }
}

std::vector<double> MathUtils::normalizeVector(const std::vector<double>& vec) {
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

std::vector<double> MathUtils::quaternionToEuler(const geometry_msgs::msg::Quaternion& q) {
    tf2::Quaternion tf_quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_quat);
    
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    return {roll, pitch, yaw};
}

geometry_msgs::msg::Quaternion MathUtils::eulerToQuaternion(double roll, double pitch, double yaw) {
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(roll, pitch, yaw);
    
    geometry_msgs::msg::Quaternion q;
    q.x = tf_quat.x();
    q.y = tf_quat.y();
    q.z = tf_quat.z();
    q.w = tf_quat.w();
    
    return q;
}

double MathUtils::lerp(double a, double b, double t) {
    return a + t * (b - a);
}

geometry_msgs::msg::Pose MathUtils::lerpPose(const geometry_msgs::msg::Pose& a, 
                                           const geometry_msgs::msg::Pose& b, 
                                           double t) {
    geometry_msgs::msg::Pose result;
    
    // Interpolate position
    result.position.x = lerp(a.position.x, b.position.x, t);
    result.position.y = lerp(a.position.y, b.position.y, t);
    result.position.z = lerp(a.position.z, b.position.z, t);
    
    // Interpolate orientation (SLERP)
    tf2::Quaternion qa(a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w);
    tf2::Quaternion qb(b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w);
    
    tf2::Quaternion qr = qa.slerp(qb, t);
    result.orientation.x = qr.x();
    result.orientation.y = qr.y();
    result.orientation.z = qr.z();
    result.orientation.w = qr.w();
    
    return result;
}

double MathUtils::sigmoid(double x) {
    return 1.0 / (1.0 + std::exp(-x));
}

Eigen::MatrixXd MathUtils::covarianceToEigen(const std::vector<double>& covariance, int rows) {
    Eigen::MatrixXd matrix(rows, rows);
    
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < rows; ++j) {
            matrix(i, j) = covariance[i * rows + j];
        }
    }
    
    return matrix;
}

} // namespace util
} // namespace decentralized_auction_mm