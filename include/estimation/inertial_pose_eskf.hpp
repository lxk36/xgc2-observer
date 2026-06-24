#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "geometry/se3.hpp"

namespace xgc2_math {

struct InertialPoseEskfConfig {
    double gravity_mps2{9.8066};
    Pose3 measurement_frame_to_world{};
    Pose3 body_to_marker{};
    bool estimate_extrinsic{false};

    double accel_noise_std{0.35};
    double gyro_noise_std{0.03};
    double pose_position_noise_std{0.01};
    double pose_orientation_noise_std{0.01};
    double gyro_bias_random_walk_std{1.0e-4};
    double accel_bias_random_walk_std{1.0e-3};
    double extrinsic_position_random_walk_std{1.0e-5};
    double extrinsic_orientation_random_walk_std{1.0e-5};
    double innovation_position_gate_m{1.5};
    double innovation_orientation_gate_rad{0.8};
    double covariance_high_threshold{100.0};
    double max_propagation_dt_s{0.05};
    double initial_position_variance{0.01};
    double initial_velocity_variance{0.1};
    double initial_orientation_variance{0.01};
    double initial_gyro_bias_variance{0.01};
    double initial_accel_bias_variance{0.1};
    double initial_extrinsic_position_variance{1.0e-8};
    double initial_extrinsic_orientation_variance{1.0e-8};
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

inline double nonNegativeOr(double value, double fallback) {
    return std::isfinite(value) && value >= 0.0 ? value : fallback;
}

inline bool validInertialSample(const InertialSample& sample) {
    return sample.received && sample.valid && std::isfinite(sample.stamp_sec) && isFinite(sample.angular_velocity) &&
           isFinite(sample.linear_acceleration);
}

inline bool validPoseMeasurement(const PoseMeasurement& sample) {
    return sample.received && sample.valid && std::isfinite(sample.stamp_sec) && isFinite(sample.pose);
}

inline Eigen::Matrix3d skewMatrix(const Eigen::Vector3d& value) {
    Eigen::Matrix3d result;
    result(0, 0) = 0.0;
    result(0, 1) = -value.z();
    result(0, 2) = value.y();
    result(1, 0) = value.z();
    result(1, 1) = 0.0;
    result(1, 2) = -value.x();
    result(2, 0) = -value.y();
    result(2, 1) = value.x();
    result(2, 2) = 0.0;
    return result;
}

inline Eigen::Matrix<double, 6, 1> normalizedInnovation(Eigen::Matrix<double, 6, 1> value) {
    value.tail<3>() = logMap(expMap(value.tail<3>()));
    return value;
}

} // namespace inertial_pose_eskf_detail

inline void normalize(InertialPoseEskfConfig& config) {
    config.gravity_mps2 = inertial_pose_eskf_detail::positiveOr(config.gravity_mps2, 9.8066);
    config.measurement_frame_to_world.orientation = normalizedQuaternion(config.measurement_frame_to_world.orientation);
    config.body_to_marker.orientation = normalizedQuaternion(config.body_to_marker.orientation);
    config.accel_noise_std = inertial_pose_eskf_detail::positiveOr(config.accel_noise_std, 0.35);
    config.gyro_noise_std = inertial_pose_eskf_detail::positiveOr(config.gyro_noise_std, 0.03);
    config.pose_position_noise_std = inertial_pose_eskf_detail::positiveOr(config.pose_position_noise_std, 0.01);
    config.pose_orientation_noise_std = inertial_pose_eskf_detail::positiveOr(config.pose_orientation_noise_std, 0.01);
    config.gyro_bias_random_walk_std =
        inertial_pose_eskf_detail::nonNegativeOr(config.gyro_bias_random_walk_std, 1.0e-4);
    config.accel_bias_random_walk_std =
        inertial_pose_eskf_detail::nonNegativeOr(config.accel_bias_random_walk_std, 1.0e-3);
    config.extrinsic_position_random_walk_std =
        inertial_pose_eskf_detail::nonNegativeOr(config.extrinsic_position_random_walk_std, 1.0e-5);
    config.extrinsic_orientation_random_walk_std =
        inertial_pose_eskf_detail::nonNegativeOr(config.extrinsic_orientation_random_walk_std, 1.0e-5);
    config.innovation_position_gate_m = inertial_pose_eskf_detail::positiveOr(config.innovation_position_gate_m, 1.5);
    config.innovation_orientation_gate_rad =
        inertial_pose_eskf_detail::positiveOr(config.innovation_orientation_gate_rad, 0.8);
    config.covariance_high_threshold = inertial_pose_eskf_detail::positiveOr(config.covariance_high_threshold, 100.0);
    config.max_propagation_dt_s = inertial_pose_eskf_detail::positiveOr(config.max_propagation_dt_s, 0.05);
    config.initial_position_variance = inertial_pose_eskf_detail::positiveOr(config.initial_position_variance, 0.01);
    config.initial_velocity_variance = inertial_pose_eskf_detail::positiveOr(config.initial_velocity_variance, 0.1);
    config.initial_orientation_variance =
        inertial_pose_eskf_detail::positiveOr(config.initial_orientation_variance, 0.01);
    config.initial_gyro_bias_variance = inertial_pose_eskf_detail::positiveOr(config.initial_gyro_bias_variance, 0.01);
    config.initial_accel_bias_variance = inertial_pose_eskf_detail::positiveOr(config.initial_accel_bias_variance, 0.1);
    config.initial_extrinsic_position_variance =
        config.estimate_extrinsic
            ? inertial_pose_eskf_detail::positiveOr(config.initial_extrinsic_position_variance, 1.0e-4)
            : 1.0e-12;
    config.initial_extrinsic_orientation_variance =
        config.estimate_extrinsic
            ? inertial_pose_eskf_detail::positiveOr(config.initial_extrinsic_orientation_variance, 1.0e-4)
            : 1.0e-12;
}

inline InertialPoseEskfConfig normalized(InertialPoseEskfConfig config) {
    normalize(config);
    return config;
}

class InertialPoseEskf {
  public:
    static constexpr int kErrorStateDim = 21;
    using ErrorVector = Eigen::Matrix<double, kErrorStateDim, 1>;
    using ErrorCovariance = Eigen::Matrix<double, kErrorStateDim, kErrorStateDim>;
    using MeasurementVector = Eigen::Matrix<double, 6, 1>;
    using MeasurementMatrix = Eigen::Matrix<double, 6, kErrorStateDim>;
    using MeasurementCovariance = Eigen::Matrix<double, 6, 6>;

    struct PoseUpdateResult {
        bool accepted{false};
        bool innovation_rejected{false};
        double position_innovation_norm{0.0};
        double orientation_innovation_norm{0.0};
        double mahalanobis_distance{0.0};
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
        resetCovariance();
        corrected_body_pose_ = Pose3{};
        has_corrected_body_pose_ = false;
    }

    void initializeFromPose(const PoseMeasurement& pose, const InertialSample* inertial = nullptr) {
        if (!inertial_pose_eskf_detail::validPoseMeasurement(pose)) {
            return;
        }

        resetCovariance();
        const Pose3 marker_world = markerPoseInWorld(pose.pose);
        const Pose3 body_world = bodyPoseFromMarkerPose(marker_world, config_.body_to_marker);
        state_.position = body_world.position;
        state_.velocity = Eigen::Vector3d::Zero();
        state_.orientation = normalizedQuaternion(body_world.orientation);
        state_.gravity = Eigen::Vector3d(0.0, 0.0, -config_.gravity_mps2);
        state_.body_to_marker = config_.body_to_marker;
        state_.last_pose_stamp_sec = pose.stamp_sec;
        state_.last_inertial_stamp_sec =
            inertial != nullptr && inertial_pose_eskf_detail::validInertialSample(*inertial) ? inertial->stamp_sec
                                                                                             : pose.stamp_sec;
        if (inertial != nullptr && inertial_pose_eskf_detail::validInertialSample(*inertial)) {
            state_.angular_velocity = inertial->angular_velocity - state_.gyro_bias;
        }
        state_.covariance_trace = covariance_.trace();
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
        if (!std::isfinite(dt) || dt <= kMinDt) {
            state_.last_inertial_stamp_sec = inertial.stamp_sec;
            return;
        }
        if (dt > config_.max_propagation_dt_s) {
            state_.last_inertial_stamp_sec = inertial.stamp_sec;
            inflateCovariance(config_.max_propagation_dt_s);
            return;
        }

        const Eigen::Vector3d accel_body = inertial.linear_acceleration - state_.accel_bias;
        const Eigen::Quaterniond old_orientation = state_.orientation;
        const Eigen::Matrix3d rotation = old_orientation.toRotationMatrix();
        const Eigen::Vector3d acceleration_world = rotation * accel_body + state_.gravity;
        state_.position += state_.velocity * dt + 0.5 * acceleration_world * dt * dt;
        state_.velocity += acceleration_world * dt;
        state_.orientation = normalizedQuaternion(state_.orientation * expMap(state_.angular_velocity * dt));
        state_.last_inertial_stamp_sec = inertial.stamp_sec;
        propagateCovariance(accel_body, rotation, dt);
        state_.covariance_trace = covariance_.trace();
    }

    PoseUpdateResult updatePose(const PoseMeasurement& pose) {
        PoseUpdateResult result;
        if (!inertial_pose_eskf_detail::validPoseMeasurement(pose)) {
            return result;
        }

        const Pose3 marker_world = markerPoseInWorld(pose.pose);
        const Pose3 predicted_marker = predictedMarkerPose(state_);
        const MeasurementVector innovation = measurementResidual(predicted_marker, marker_world);
        result.position_innovation_norm = innovation.head<3>().norm();
        result.orientation_innovation_norm = innovation.tail<3>().norm();
        if (result.position_innovation_norm > config_.innovation_position_gate_m ||
            result.orientation_innovation_norm > config_.innovation_orientation_gate_rad) {
            result.innovation_rejected = true;
            state_.last_pose_stamp_sec = pose.stamp_sec;
            return result;
        }

        if (!state_.initialized) {
            corrected_body_pose_ = bodyPoseFromMarkerPose(marker_world, config_.body_to_marker);
            has_corrected_body_pose_ = true;
            return result;
        }

        const MeasurementMatrix H = numericalMeasurementJacobian(predicted_marker, marker_world);
        const MeasurementCovariance R = measurementCovariance();
        const MeasurementCovariance S = H * covariance_ * H.transpose() + R;
        const Eigen::LDLT<MeasurementCovariance> ldlt(S);
        if (ldlt.info() != Eigen::Success || !S.allFinite()) {
            return result;
        }

        result.mahalanobis_distance = innovation.transpose() * ldlt.solve(innovation);
        const Eigen::Matrix<double, kErrorStateDim, 6> K =
            covariance_ * H.transpose() * ldlt.solve(MeasurementCovariance::Identity());
        const ErrorVector delta = K * innovation;
        injectError(delta);

        const ErrorCovariance identity = ErrorCovariance::Identity();
        covariance_ = (identity - K * H) * covariance_ * (identity - K * H).transpose() + K * R * K.transpose();
        covariance_ = 0.5 * (covariance_ + covariance_.transpose());
        state_.last_pose_stamp_sec = pose.stamp_sec;
        state_.covariance_trace = covariance_.trace();
        corrected_body_pose_ = bodyPoseFromMarkerPose(marker_world, state_.body_to_marker);
        has_corrected_body_pose_ = true;
        result.accepted = true;
        return result;
    }

    const RigidBodyState& state() const { return state_; }
    const ErrorCovariance& covariance() const { return covariance_; }
    bool initialized() const { return state_.initialized; }
    bool hasCorrectedBodyPose() const { return has_corrected_body_pose_; }
    const Pose3& correctedBodyPose() const { return corrected_body_pose_; }

  private:
    void resetCovariance() {
        covariance_.setIdentity();
        covariance_ *= 1.0e-6;
        covariance_.block<3, 3>(0, 0).diagonal().setConstant(config_.initial_position_variance);
        covariance_.block<3, 3>(3, 3).diagonal().setConstant(config_.initial_velocity_variance);
        covariance_.block<3, 3>(6, 6).diagonal().setConstant(config_.initial_orientation_variance);
        covariance_.block<3, 3>(9, 9).diagonal().setConstant(config_.initial_gyro_bias_variance);
        covariance_.block<3, 3>(12, 12).diagonal().setConstant(config_.initial_accel_bias_variance);
        covariance_.block<3, 3>(15, 15).diagonal().setConstant(config_.initial_extrinsic_position_variance);
        covariance_.block<3, 3>(18, 18).diagonal().setConstant(config_.initial_extrinsic_orientation_variance);
        state_.covariance_trace = covariance_.trace();
    }

    Pose3 markerPoseInWorld(const Pose3& raw_marker_pose) const {
        return compose(config_.measurement_frame_to_world, raw_marker_pose);
    }

    static Pose3 bodyPoseFromMarkerPose(const Pose3& marker_pose_world, const Pose3& body_to_marker) {
        return compose(marker_pose_world, inverse(body_to_marker));
    }

    static Pose3 predictedMarkerPose(const RigidBodyState& state) {
        Pose3 body_pose;
        body_pose.position = state.position;
        body_pose.orientation = state.orientation;
        return compose(body_pose, state.body_to_marker);
    }

    static MeasurementVector measurementResidual(const Pose3& predicted_marker, const Pose3& measured_marker) {
        return inertial_pose_eskf_detail::normalizedInnovation(se3Error(predicted_marker, measured_marker));
    }

    MeasurementCovariance measurementCovariance() const {
        MeasurementCovariance R = MeasurementCovariance::Zero();
        R.block<3, 3>(0, 0).diagonal().setConstant(config_.pose_position_noise_std * config_.pose_position_noise_std);
        R.block<3, 3>(3, 3).diagonal().setConstant(config_.pose_orientation_noise_std *
                                                   config_.pose_orientation_noise_std);
        return R;
    }

    MeasurementMatrix numericalMeasurementJacobian(const Pose3& nominal_prediction,
                                                   const Pose3& measured_marker) const {
        MeasurementMatrix H;
        H.setZero();
        const MeasurementVector nominal_residual = measurementResidual(nominal_prediction, measured_marker);
        for (int i = 0; i < kErrorStateDim; ++i) {
            ErrorVector delta = ErrorVector::Zero();
            delta(i) = kJacobianEpsilon;
            RigidBodyState perturbed = state_;
            injectError(delta, perturbed);
            const MeasurementVector perturbed_residual =
                measurementResidual(predictedMarkerPose(perturbed), measured_marker);
            H.col(i) = -(perturbed_residual - nominal_residual) / kJacobianEpsilon;
        }
        if (!config_.estimate_extrinsic) {
            H.block<6, 6>(0, 15).setZero();
        }
        return H;
    }

    void propagateCovariance(const Eigen::Vector3d& accel_body, const Eigen::Matrix3d& rotation, double dt) {
        ErrorCovariance F = ErrorCovariance::Identity();
        const Eigen::Matrix3d accel_skew = inertial_pose_eskf_detail::skewMatrix(accel_body);

        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
        F.block<3, 3>(0, 6) = -0.5 * rotation * accel_skew * dt * dt;
        F.block<3, 3>(0, 12) = -0.5 * rotation * dt * dt;
        F.block<3, 3>(3, 6) = -rotation * accel_skew * dt;
        F.block<3, 3>(3, 12) = -rotation * dt;
        F.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity() * dt;

        ErrorCovariance Q = ErrorCovariance::Zero();
        const double accel_var = config_.accel_noise_std * config_.accel_noise_std;
        const double gyro_var = config_.gyro_noise_std * config_.gyro_noise_std;
        Q.block<3, 3>(0, 0).diagonal().setConstant(0.25 * accel_var * dt * dt * dt * dt);
        Q.block<3, 3>(3, 3).diagonal().setConstant(accel_var * dt * dt);
        Q.block<3, 3>(6, 6).diagonal().setConstant(gyro_var * dt * dt);
        Q.block<3, 3>(9, 9).diagonal().setConstant(config_.gyro_bias_random_walk_std *
                                                   config_.gyro_bias_random_walk_std * dt);
        Q.block<3, 3>(12, 12).diagonal().setConstant(config_.accel_bias_random_walk_std *
                                                     config_.accel_bias_random_walk_std * dt);
        if (config_.estimate_extrinsic) {
            Q.block<3, 3>(15, 15).diagonal().setConstant(config_.extrinsic_position_random_walk_std *
                                                         config_.extrinsic_position_random_walk_std * dt);
            Q.block<3, 3>(18, 18).diagonal().setConstant(config_.extrinsic_orientation_random_walk_std *
                                                         config_.extrinsic_orientation_random_walk_std * dt);
        }

        covariance_ = F * covariance_ * F.transpose() + Q;
        covariance_ = 0.5 * (covariance_ + covariance_.transpose());
    }

    void inflateCovariance(double dt) {
        ErrorCovariance Q = ErrorCovariance::Zero();
        Q.block<3, 3>(0, 0).diagonal().setConstant(config_.accel_noise_std * config_.accel_noise_std * dt * dt);
        Q.block<3, 3>(3, 3).diagonal().setConstant(config_.accel_noise_std * config_.accel_noise_std * dt);
        Q.block<3, 3>(6, 6).diagonal().setConstant(config_.gyro_noise_std * config_.gyro_noise_std * dt);
        covariance_ += Q;
        covariance_ = 0.5 * (covariance_ + covariance_.transpose());
        state_.covariance_trace = covariance_.trace();
    }

    void injectError(const ErrorVector& delta) { injectError(delta, state_); }

    void injectError(const ErrorVector& delta, RigidBodyState& state) const {
        state.position += delta.segment<3>(0);
        state.velocity += delta.segment<3>(3);
        state.orientation = normalizedQuaternion(state.orientation * expMap(delta.segment<3>(6)));
        state.gyro_bias += delta.segment<3>(9);
        state.accel_bias += delta.segment<3>(12);
        if (config_.estimate_extrinsic) {
            state.body_to_marker.position += delta.segment<3>(15);
            state.body_to_marker.orientation =
                normalizedQuaternion(state.body_to_marker.orientation * expMap(delta.segment<3>(18)));
        }
    }

    static constexpr double kMinDt = 1.0e-5;
    static constexpr double kJacobianEpsilon = 1.0e-6;

    InertialPoseEskfConfig config_{};
    RigidBodyState state_{};
    ErrorCovariance covariance_{ErrorCovariance::Identity()};
    Pose3 corrected_body_pose_{};
    bool has_corrected_body_pose_{false};
};

} // namespace xgc2_math
