#ifndef DECENTRALIZED_AUCTION_MM_MATH_UTILS_HPP
#define DECENTRALIZED_AUCTION_MM_MATH_UTILS_HPP

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace decentralized_auction_mm {
namespace util {

/**
 * @class MathUtils
 * @brief Utility functions for mathematical operations
 * 
 * This class provides various mathematical utilities for
 * robot control, task allocation, and optimization.
 */
class MathUtils {
public:
    /**
     * @brief Calculate Euclidean distance between two points
     * @param p1 First point
     * @param p2 Second point
     * @return Euclidean distance
     */
    static double euclideanDistance(const geometry_msgs::msg::Point& p1, 
                                   const geometry_msgs::msg::Point& p2);

    /**
     * @brief Calculate Manhattan distance between two points
     * @param p1 First point
     * @param p2 Second point
     * @return Manhattan distance
     */
    static double manhattanDistance(const geometry_msgs::msg::Point& p1, 
                                   const geometry_msgs::msg::Point& p2);

    /**
     * @brief Calculate cosine similarity between two vectors
     * @param v1 First vector
     * @param v2 Second vector
     * @return Cosine similarity
     */
    static double cosineSimilarity(const std::vector<double>& v1, 
                                  const std::vector<double>& v2);

    /**
     * @brief Normalize a vector to unit length
     * @param vec Vector to normalize
     * @return Normalized vector
     */
    static std::vector<double> normalizeVector(const std::vector<double>& vec);

    /**
     * @brief Convert quaternion to Euler angles
     * @param q Quaternion
     * @return Vector of Euler angles [roll, pitch, yaw]
     */
    static std::vector<double> quaternionToEuler(const geometry_msgs::msg::Quaternion& q);

    /**
     * @brief Convert Euler angles to quaternion
     * @param roll Roll angle
     * @param pitch Pitch angle
     * @param yaw Yaw angle
     * @return Quaternion
     */
    static geometry_msgs::msg::Quaternion eulerToQuaternion(double roll, double pitch, double yaw);

    /**
     * @brief Linear interpolation between two values
     * @param a First value
     * @param b Second value
     * @param t Interpolation parameter (0 to 1)
     * @return Interpolated value
     */
    static double lerp(double a, double b, double t);

    /**
     * @brief Linear interpolation between two poses
     * @param a First pose
     * @param b Second pose
     * @param t Interpolation parameter (0 to 1)
     * @return Interpolated pose
     */
    static geometry_msgs::msg::Pose lerpPose(const geometry_msgs::msg::Pose& a, 
                                           const geometry_msgs::msg::Pose& b, 
                                           double t);

    /**
     * @brief Calculate sigmoid function
     * @param x Input value
     * @return Sigmoid value
     */
    static double sigmoid(double x);

    /**
     * @brief Convert covariance matrix to Eigen matrix
     * @param covariance Covariance array (row-major)
     * @param rows Number of rows
     * @return Eigen matrix
     */
    static Eigen::MatrixXd covarianceToEigen(const std::vector<double>& covariance, int rows);
};

} // namespace util
} // namespace decentralized_auction_mm

#endif // DECENTRALIZED_AUCTION_MM_MATH_UTILS_HPP