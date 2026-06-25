#ifndef XGC2_MATH_GEOMETRY_GEOMETRY_MATH_HELPERS_H
#define XGC2_MATH_GEOMETRY_GEOMETRY_MATH_HELPERS_H

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

namespace xgc2_math {
namespace math_helpers {

constexpr double kDirectionEpsilon = 1e-12;

inline double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(value, max_value));
}

inline Eigen::Vector3d normalizedOrZero(const Eigen::Vector3d& value, double direction_epsilon = kDirectionEpsilon) {
    if (!value.array().isFinite().all() || value.squaredNorm() <= direction_epsilon) {
        return Eigen::Vector3d::Zero();
    }
    return value.normalized();
}

inline bool isFiniteVector3(const Eigen::Vector3d& value) {
    return value.array().isFinite().all();
}

inline void quaternionToEuler(double qx, double qy, double qz, double qw, double& roll, double& pitch, double& yaw) {
    const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(M_PI / 2.0, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

inline double quaternionToYaw(double qx, double qy, double qz, double qw) {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    quaternionToEuler(qx, qy, qz, qw, roll, pitch, yaw);
    return yaw;
}

inline void eulerToQuaternion(double roll, double pitch, double yaw, double& qx, double& qy, double& qz, double& qw) {
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);

    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
}

inline double sign(double value) {
    constexpr double kEpsilon = 1e-10;
    if (value > kEpsilon) {
        return 1.0;
    }
    if (value < -kEpsilon) {
        return -1.0;
    }
    return 0.0;
}

inline double signBeta(double value, double beta) {
    constexpr double kEpsilon = 1e-10;
    if (std::abs(beta) <= kEpsilon) {
        return sign(value);
    }
    if (beta > kEpsilon) {
        return sign(value) * std::pow(std::abs(value), beta);
    }
    return 0.0;
}

} // namespace math_helpers
} // namespace xgc2_math

#endif // XGC2_MATH_GEOMETRY_GEOMETRY_MATH_HELPERS_H
