#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "xgc2_observer/se3.hpp"

namespace xgc2_observer {

struct InertialPoseEskfConfig {
    double gravity_mps2{9.8066};
    Pose3 measurement_frame_to_world{};
    Pose3 body_to_marker{};
    bool estimate_extrinsic{false};

    double accel_noise_std{0.35};
    double gyro_noise_std{0.03};
    double pose_position_noise_std{0.01};
    double pose_orientation_noise_std{0.01};
    double position_update_gain{0.85};
    double velocity_update_gain{0.25};
    double orientation_update_gain{0.85};
    double gyro_bias_update_gain{0.002};
    double accel_bias_update_gain{0.0};
    double innovation_position_gate_m{1.5};
    double innovation_orientation_gate_rad{0.8};
    double covariance_high_threshold{100.0};
};

struct InertialSample {
    bool received{false};
    bool valid{false};
    bool time_jump{false};
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
    double stamp_sec{0.0};
    double last_dt_sec{0.0};
    double estimated_rate_hz{0.0};
};

struct PoseMeasurement {
    bool received{false};
    bool valid{false};
    bool time_jump{false};
    Pose3 pose{};
    double stamp_sec{0.0};
    double last_dt_sec{0.0};
    double estimated_rate_hz{0.0};
};

struct RigidBodyState {
    bool initialized{false};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
    Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
    Eigen::Vector3d gravity{0.0, 0.0, -9.8066};
    Pose3 body_to_marker{};
    double last_inertial_stamp_sec{0.0};
    double last_pose_stamp_sec{0.0};
    double covariance_trace{1.0};
};

namespace inertial_pose_eskf_detail {

inline double positiveOr(double value, double fallback) {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}

inline double clamp01(double value) {
    if (!std::isfinite(value)) {
        return 0.0;
    }
    return std::clamp(value, 0.0, 1.0);
}

inline bool validInertialSample(const InertialSample& sample) {
    return sample.received && sample.valid && std::isfinite(sample.stamp_sec) && isFinite(sample.angular_velocity) &&
           isFinite(sample.linear_acceleration);
}

inline bool validPoseMeasurement(const PoseMeasurement& sample) {
    return sample.received && sample.valid && std::isfinite(sample.stamp_sec) && isFinite(sample.pose);
}

} // namespace inertial_pose_eskf_detail

inline void normalize(InertialPoseEskfConfig& config) {
    config.gravity_mps2 = inertial_pose_eskf_detail::positiveOr(config.gravity_mps2, 9.8066);
    config.accel_noise_std = inertial_pose_eskf_detail::positiveOr(config.accel_noise_std, 0.35);
    config.gyro_noise_std = inertial_pose_eskf_detail::positiveOr(config.gyro_noise_std, 0.03);
    config.pose_position_noise_std = inertial_pose_eskf_detail::positiveOr(config.pose_position_noise_std, 0.01);
    config.pose_orientation_noise_std = inertial_pose_eskf_detail::positiveOr(config.pose_orientation_noise_std, 0.01);
    config.position_update_gain = inertial_pose_eskf_detail::clamp01(config.position_update_gain);
    config.velocity_update_gain = inertial_pose_eskf_detail::clamp01(config.velocity_update_gain);
    config.orientation_update_gain = inertial_pose_eskf_detail::clamp01(config.orientation_update_gain);
    config.gyro_bias_update_gain = inertial_pose_eskf_detail::clamp01(config.gyro_bias_update_gain);
    config.accel_bias_update_gain = inertial_pose_eskf_detail::clamp01(config.accel_bias_update_gain);
    config.innovation_position_gate_m = inertial_pose_eskf_detail::positiveOr(config.innovation_position_gate_m, 1.5);
    config.innovation_orientation_gate_rad =
        inertial_pose_eskf_detail::positiveOr(config.innovation_orientation_gate_rad, 0.8);
    config.covariance_high_threshold = inertial_pose_eskf_detail::positiveOr(config.covariance_high_threshold, 100.0);
    config.measurement_frame_to_world.orientation = normalizedQuaternion(config.measurement_frame_to_world.orientation);
    config.body_to_marker.orientation = normalizedQuaternion(config.body_to_marker.orientation);
}

inline InertialPoseEskfConfig normalized(InertialPoseEskfConfig config) {
    normalize(config);
    return config;
}

class InertialPoseEskf {
  public:
    struct PoseUpdateResult {
        bool accepted{false};
        bool innovation_rejected{false};
        double position_innovation_norm{0.0};
        double orientation_innovation_norm{0.0};
    };

    InertialPoseEskf() { reset(); }

    void setConfig(const InertialPoseEskfConfig& config) {
        config_ = normalized(config);
        reset();
    }

    void reset() {
        state_ = RigidBodyState{};
        state_.gravity = Eigen::Vector3d(0.0, 0.0, -config_.gravity_mps2);
        state_.body_to_marker = config_.body_to_marker;
        state_.covariance_trace = kInitialCovarianceTrace;
        corrected_body_pose_ = Pose3{};
        has_corrected_body_pose_ = false;
    }

    void initializeFromPose(const PoseMeasurement& pose, const InertialSample* inertial) {
        if (!inertial_pose_eskf_detail::validPoseMeasurement(pose)) {
            return;
        }

        const Pose3 marker_world = markerPoseInWorld(pose.pose);
        const Pose3 body_world = bodyPoseFromMarkerPose(marker_world);
        state_.position = body_world.position;
        state_.velocity = Eigen::Vector3d::Zero();
        state_.orientation = normalizedQuaternion(body_world.orientation);
        state_.gravity = Eigen::Vector3d(0.0, 0.0, -config_.gravity_mps2);
        state_.body_to_marker = config_.body_to_marker;
        state_.last_pose_stamp_sec = pose.stamp_sec;
        if (inertial != nullptr && inertial_pose_eskf_detail::validInertialSample(*inertial)) {
            state_.angular_velocity = inertial->angular_velocity - state_.gyro_bias;
            state_.last_inertial_stamp_sec = inertial->stamp_sec;
        }
        state_.covariance_trace = 0.1;
        state_.initialized = true;
        corrected_body_pose_ = body_world;
        has_corrected_body_pose_ = true;
    }

    void propagateInertial(const InertialSample& inertial) {
        if (!inertial_pose_eskf_detail::validInertialSample(inertial)) {
            return;
        }

        state_.angular_velocity = inertial.angular_velocity - state_.gyro_bias;

        if (!state_.initialized) {
            state_.last_inertial_stamp_sec = inertial.stamp_sec;
            return;
        }

        const double dt = inertial.stamp_sec - state_.last_inertial_stamp_sec;
        if (!std::isfinite(dt) || dt <= kMinDt || dt > kMaxPropagationDt) {
            state_.last_inertial_stamp_sec = inertial.stamp_sec;
            return;
        }

        const Eigen::Quaterniond delta_q = expMap(state_.angular_velocity * dt);
        state_.orientation = normalizedQuaternion(state_.orientation * delta_q);

        const Eigen::Vector3d specific_force = inertial.linear_acceleration - state_.accel_bias;
        const Eigen::Vector3d acceleration_world = state_.orientation * specific_force + state_.gravity;
        state_.position += state_.velocity * dt + 0.5 * acceleration_world * dt * dt;
        state_.velocity += acceleration_world * dt;
        state_.last_inertial_stamp_sec = inertial.stamp_sec;
        inflateCovariance(dt);
    }

    PoseUpdateResult updatePose(const PoseMeasurement& pose) {
        PoseUpdateResult result;
        if (!inertial_pose_eskf_detail::validPoseMeasurement(pose)) {
            return result;
        }

        const Pose3 marker_world = markerPoseInWorld(pose.pose);
        const Pose3 corrected_body_pose = bodyPoseFromMarkerPose(marker_world);

        if (!state_.initialized) {
            corrected_body_pose_ = corrected_body_pose;
            has_corrected_body_pose_ = true;
            return result;
        }

        const Pose3 predicted_marker = predictedMarkerPose();
        const Eigen::Vector3d position_residual = marker_world.position - predicted_marker.position;
        const Eigen::Quaterniond orientation_residual =
            normalizedQuaternion(predicted_marker.orientation.conjugate() * marker_world.orientation);
        const Eigen::Vector3d rotation_residual = logMap(orientation_residual);

        result.position_innovation_norm = position_residual.norm();
        result.orientation_innovation_norm = rotation_residual.norm();

        if (result.position_innovation_norm > config_.innovation_position_gate_m ||
            result.orientation_innovation_norm > config_.innovation_orientation_gate_rad) {
            result.innovation_rejected = true;
            state_.last_pose_stamp_sec = pose.stamp_sec;
            return result;
        }

        const double dt_pose = pose.stamp_sec - state_.last_pose_stamp_sec;
        const double velocity_dt = std::isfinite(dt_pose) && dt_pose > kMinDt ? dt_pose : 0.0;

        state_.position += config_.position_update_gain * position_residual;
        if (velocity_dt > 0.0) {
            state_.velocity += config_.velocity_update_gain * (position_residual / std::max(velocity_dt, 0.01));
        }
        state_.orientation =
            normalizedQuaternion(state_.orientation * expMap(config_.orientation_update_gain * rotation_residual));

        if (velocity_dt > 0.0 && config_.gyro_bias_update_gain > 0.0) {
            const Eigen::Vector3d bias_step =
                clampNorm(-config_.gyro_bias_update_gain * rotation_residual / velocity_dt, kMaxBiasCorrection);
            state_.gyro_bias += bias_step;
        }

        state_.last_pose_stamp_sec = pose.stamp_sec;
        corrected_body_pose_ = corrected_body_pose;
        has_corrected_body_pose_ = true;
        reduceCovariance();
        result.accepted = true;
        return result;
    }

    const RigidBodyState& state() const { return state_; }
    bool initialized() const { return state_.initialized; }
    bool hasCorrectedBodyPose() const { return has_corrected_body_pose_; }
    const Pose3& correctedBodyPose() const { return corrected_body_pose_; }

  private:
    Pose3 markerPoseInWorld(const Pose3& raw_marker_pose) const {
        return compose(config_.measurement_frame_to_world, raw_marker_pose);
    }

    Pose3 bodyPoseFromMarkerPose(const Pose3& marker_pose_world) const {
        return compose(marker_pose_world, inverse(config_.body_to_marker));
    }

    Pose3 predictedMarkerPose() const {
        Pose3 body_pose;
        body_pose.position = state_.position;
        body_pose.orientation = state_.orientation;
        return compose(body_pose, config_.body_to_marker);
    }

    void reduceCovariance() { state_.covariance_trace = std::max(kMinCovarianceTrace, state_.covariance_trace * 0.5); }

    void inflateCovariance(double dt_sec) {
        const double process_noise =
            config_.accel_noise_std * config_.accel_noise_std + config_.gyro_noise_std * config_.gyro_noise_std;
        state_.covariance_trace += std::max(0.0, dt_sec) * process_noise;
    }

    static constexpr double kMaxPropagationDt = 0.05;
    static constexpr double kMinDt = 1.0e-5;
    static constexpr double kMaxBiasCorrection = 0.01;
    static constexpr double kMinCovarianceTrace = 1.0e-4;
    static constexpr double kInitialCovarianceTrace = 1.0;

    InertialPoseEskfConfig config_{};
    RigidBodyState state_{};
    Pose3 corrected_body_pose_{};
    bool has_corrected_body_pose_{false};
};

} // namespace xgc2_observer
