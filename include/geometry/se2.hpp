#pragma once

#include <cmath>

#include <Eigen/Core>

#include "core/angle.hpp"

namespace xgc2_math {

struct Pose2 {
    Eigen::Vector2d position{Eigen::Vector2d::Zero()};
    double yaw{0.0};
};

struct PlanarRigidBodyState {
    bool initialized{false};
    Eigen::Vector2d position{Eigen::Vector2d::Zero()};
    Eigen::Vector2d velocity{Eigen::Vector2d::Zero()};
    double yaw{0.0};
    double yaw_rate{0.0};
    double gyro_bias_z{0.0};
    Eigen::Vector2d accel_bias{Eigen::Vector2d::Zero()};
    Pose2 body_to_marker{};
    double last_inertial_stamp_sec{0.0};
    double last_pose_stamp_sec{0.0};
    double covariance_trace{1.0};
};

inline bool isFinite(const Eigen::Vector2d& value) {
    return value.allFinite();
}

inline bool isFinite(const Pose2& pose) {
    return isFinite(pose.position) && std::isfinite(pose.yaw);
}

inline Eigen::Matrix2d rotationMatrix2(double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    Eigen::Matrix2d result;
    result(0, 0) = c;
    result(0, 1) = -s;
    result(1, 0) = s;
    result(1, 1) = c;
    return result;
}

inline Pose2 normalized(Pose2 pose) {
    if (!isFinite(pose)) {
        return Pose2{};
    }
    pose.yaw = normalizeAngle(pose.yaw);
    return pose;
}

inline Pose2 compose(const Pose2& lhs, const Pose2& rhs) {
    Pose2 result;
    const Pose2 lhs_norm = normalized(lhs);
    result.position = lhs_norm.position + rotationMatrix2(lhs_norm.yaw) * rhs.position;
    result.yaw = normalizeAngle(lhs_norm.yaw + rhs.yaw);
    return result;
}

inline Pose2 inverse(const Pose2& pose) {
    Pose2 result;
    const Pose2 pose_norm = normalized(pose);
    result.yaw = normalizeAngle(-pose_norm.yaw);
    result.position = -(rotationMatrix2(result.yaw) * pose_norm.position);
    return result;
}

inline Eigen::Vector3d se2Error(const Pose2& reference, const Pose2& measured) {
    const Pose2 delta = compose(inverse(reference), measured);
    return Eigen::Vector3d(delta.position.x(), delta.position.y(), normalizeAngle(delta.yaw));
}

} // namespace xgc2_math
