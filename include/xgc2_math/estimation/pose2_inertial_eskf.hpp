#pragma once

#include <algorithm>
#include <cstddef>
#include <cmath>

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include "xgc2_math/estimation/health.hpp"
#include "xgc2_math/geometry/se2.hpp"

namespace xgc2_math {

struct Pose2InertialEskfConfig {
    Pose2 measurement_frame_to_world{};
    Pose2 body_to_marker{};
    bool estimate_extrinsic{false};

    double gyro_noise_std{0.03};
    double accel_noise_std{0.35};
    double gyro_bias_random_walk_std{1.0e-4};
    double accel_bias_random_walk_std{1.0e-3};
    double extrinsic_position_random_walk_std{1.0e-5};
    double extrinsic_yaw_random_walk_std{1.0e-5};
    double pose_position_noise_std{0.01};
    double pose_yaw_noise_std{0.01};
    double innovation_position_gate_m{1.5};
    double innovation_yaw_gate_rad{0.8};
    double covariance_high_threshold{100.0};
    double max_propagation_dt_s{0.05};
    double initial_position_variance{0.01};
    double initial_velocity_variance{0.1};
    double initial_yaw_variance{0.01};
    double initial_gyro_bias_variance{0.01};
    double initial_accel_bias_variance{0.1};
    double initial_extrinsic_position_variance{1.0e-8};
    double initial_extrinsic_yaw_variance{1.0e-8};
    std::size_t inertial_buffer_capacity{512};
    ObservationHealthConfig vrpn_health{};
};

struct PlanarInertialSample {
    bool received{false};
    bool valid{false};
    bool time_jump{false};
    double stamp_sec{0.0};
    double last_dt_sec{0.0};
    double estimated_rate_hz{0.0};
    double angular_velocity_z{0.0};
    Eigen::Vector2d linear_acceleration{Eigen::Vector2d::Zero()};
};

struct PlanarPoseMeasurement {
    bool received{false};
    bool valid{false};
    bool time_jump{false};
    Pose2 pose{};
    double stamp_sec{0.0};
    double last_dt_sec{0.0};
    double estimated_rate_hz{0.0};
};

struct PlanarInertialPropagationResult {
    bool accepted{false};
    bool initialized_stamp{false};
};

struct PlanarPoseUpdateResult {
    bool accepted{false};
    bool innovation_rejected{false};
    bool time_alignment_rejected{false};
    double position_innovation_norm{0.0};
    double yaw_innovation_abs{0.0};
    double mahalanobis_distance{0.0};
    double innovation_window_chi_square{0.0};
    VrpnObservationState vrpn_observation_state{VrpnObservationState::kTrusted};
    FilterHealth filter_health{FilterHealth::kLost};
    PoseFusionRejectReason reject_reason{PoseFusionRejectReason::kNone};
};

namespace pose2_inertial_eskf_detail {

inline double positiveOr(double value, double fallback) {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}

inline double nonNegativeOr(double value, double fallback) {
    return std::isfinite(value) && value >= 0.0 ? value : fallback;
}

inline bool validPoseMeasurement(const PlanarPoseMeasurement& sample) {
    return sample.received && sample.valid && std::isfinite(sample.stamp_sec) && isFinite(sample.pose);
}

inline bool validInertialSample(const PlanarInertialSample& sample) {
    return sample.received && sample.valid && std::isfinite(sample.stamp_sec) &&
           std::isfinite(sample.angular_velocity_z) && isFinite(sample.linear_acceleration);
}

inline Eigen::Matrix2d yawDerivativeMatrix() {
    Eigen::Matrix2d result;
    result(0, 0) = 0.0;
    result(0, 1) = -1.0;
    result(1, 0) = 1.0;
    result(1, 1) = 0.0;
    return result;
}

inline Eigen::Vector3d normalizedInnovation(Eigen::Vector3d value) {
    value.z() = normalizeAngle(value.z());
    return value;
}

} // namespace pose2_inertial_eskf_detail

inline void normalize(Pose2InertialEskfConfig& config) {
    config.measurement_frame_to_world = normalized(config.measurement_frame_to_world);
    config.body_to_marker = normalized(config.body_to_marker);
    config.gyro_noise_std = pose2_inertial_eskf_detail::positiveOr(config.gyro_noise_std, 0.03);
    config.accel_noise_std = pose2_inertial_eskf_detail::positiveOr(config.accel_noise_std, 0.35);
    config.gyro_bias_random_walk_std =
        pose2_inertial_eskf_detail::nonNegativeOr(config.gyro_bias_random_walk_std, 1.0e-4);
    config.accel_bias_random_walk_std =
        pose2_inertial_eskf_detail::nonNegativeOr(config.accel_bias_random_walk_std, 1.0e-3);
    config.extrinsic_position_random_walk_std =
        pose2_inertial_eskf_detail::nonNegativeOr(config.extrinsic_position_random_walk_std, 1.0e-5);
    config.extrinsic_yaw_random_walk_std =
        pose2_inertial_eskf_detail::nonNegativeOr(config.extrinsic_yaw_random_walk_std, 1.0e-5);
    config.pose_position_noise_std = pose2_inertial_eskf_detail::positiveOr(config.pose_position_noise_std, 0.01);
    config.pose_yaw_noise_std = pose2_inertial_eskf_detail::positiveOr(config.pose_yaw_noise_std, 0.01);
    config.innovation_position_gate_m = pose2_inertial_eskf_detail::positiveOr(config.innovation_position_gate_m, 1.5);
    config.innovation_yaw_gate_rad = pose2_inertial_eskf_detail::positiveOr(config.innovation_yaw_gate_rad, 0.8);
    config.covariance_high_threshold = pose2_inertial_eskf_detail::positiveOr(config.covariance_high_threshold, 100.0);
    config.max_propagation_dt_s = pose2_inertial_eskf_detail::positiveOr(config.max_propagation_dt_s, 0.05);
    config.initial_position_variance = pose2_inertial_eskf_detail::positiveOr(config.initial_position_variance, 0.01);
    config.initial_velocity_variance = pose2_inertial_eskf_detail::positiveOr(config.initial_velocity_variance, 0.1);
    config.initial_yaw_variance = pose2_inertial_eskf_detail::positiveOr(config.initial_yaw_variance, 0.01);
    config.initial_gyro_bias_variance = pose2_inertial_eskf_detail::positiveOr(config.initial_gyro_bias_variance, 0.01);
    config.initial_accel_bias_variance =
        pose2_inertial_eskf_detail::positiveOr(config.initial_accel_bias_variance, 0.1);
    config.initial_extrinsic_position_variance =
        config.estimate_extrinsic
            ? pose2_inertial_eskf_detail::positiveOr(config.initial_extrinsic_position_variance, 1.0e-4)
            : 1.0e-12;
    config.initial_extrinsic_yaw_variance =
        config.estimate_extrinsic
            ? pose2_inertial_eskf_detail::positiveOr(config.initial_extrinsic_yaw_variance, 1.0e-4)
            : 1.0e-12;
}

inline Pose2InertialEskfConfig normalized(Pose2InertialEskfConfig config) {
    normalize(config);
    return config;
}

class Pose2InertialEskf {
  public:
    static constexpr int kErrorStateDim = 11;
    using ErrorVector = Eigen::Matrix<double, kErrorStateDim, 1>;
    using ErrorCovariance = Eigen::Matrix<double, kErrorStateDim, kErrorStateDim>;
    using MeasurementMatrix = Eigen::Matrix<double, 3, kErrorStateDim>;
    using MeasurementCovariance = Eigen::Matrix3d;

    void setConfig(const Pose2InertialEskfConfig& config) {
        config_ = normalized(config);
        vrpn_health_.setConfig(config_.vrpn_health);
        reset();
    }

    void reset() {
        state_ = PlanarRigidBodyState{};
        state_.body_to_marker = config_.body_to_marker;
        covariance_.setIdentity();
        covariance_ *= 1.0e-6;
        covariance_.block<2, 2>(0, 0).diagonal().setConstant(config_.initial_position_variance);
        covariance_.block<2, 2>(2, 2).diagonal().setConstant(config_.initial_velocity_variance);
        covariance_(4, 4) = config_.initial_yaw_variance;
        covariance_(5, 5) = config_.initial_gyro_bias_variance;
        covariance_.block<2, 2>(6, 6).diagonal().setConstant(config_.initial_accel_bias_variance);
        covariance_.block<2, 2>(8, 8).diagonal().setConstant(config_.initial_extrinsic_position_variance);
        covariance_(10, 10) = config_.initial_extrinsic_yaw_variance;
        corrected_body_pose_ = Pose2{};
        raw_projected_body_pose_ = Pose2{};
        has_corrected_body_pose_ = false;
        has_raw_projected_body_pose_ = false;
        vrpn_health_.reset();
        last_fused_pose_stamp_sec_ = 0.0;
    }

    void initializeFromPose(const PlanarPoseMeasurement& pose, const PlanarInertialSample* inertial = nullptr) {
        if (!pose2_inertial_eskf_detail::validPoseMeasurement(pose)) {
            return;
        }
        const Pose2 marker_world = markerPoseInWorld(pose.pose);
        const Pose2 body_world = bodyPoseFromMarkerPose(marker_world, config_.body_to_marker);
        state_.position = body_world.position;
        state_.velocity = Eigen::Vector2d::Zero();
        state_.yaw = normalizeAngle(body_world.yaw);
        state_.yaw_rate = 0.0;
        state_.gyro_bias_z = 0.0;
        state_.accel_bias = Eigen::Vector2d::Zero();
        state_.body_to_marker = config_.body_to_marker;
        state_.last_pose_stamp_sec = pose.stamp_sec;
        state_.last_inertial_stamp_sec =
            inertial != nullptr && pose2_inertial_eskf_detail::validInertialSample(*inertial) ? inertial->stamp_sec
                                                                                              : pose.stamp_sec;
        state_.covariance_trace = covariance_.trace();
        state_.initialized = true;
        corrected_body_pose_ = bodyPoseFromState(state_);
        raw_projected_body_pose_ = body_world;
        has_corrected_body_pose_ = true;
        has_raw_projected_body_pose_ = true;
        vrpn_health_.reset();
        last_fused_pose_stamp_sec_ = pose.stamp_sec;
    }

    PlanarInertialPropagationResult propagateInertial(const PlanarInertialSample& inertial) {
        PlanarInertialPropagationResult result;
        if (!pose2_inertial_eskf_detail::validInertialSample(inertial)) {
            return result;
        }

        if (!state_.initialized) {
            state_.last_inertial_stamp_sec = inertial.stamp_sec;
            result.initialized_stamp = true;
            return result;
        }

        const double dt = inertial.stamp_sec - state_.last_inertial_stamp_sec;
        if (!std::isfinite(dt) || dt <= kMinDt) {
            return result;
        }

        propagateToStamp(inertial, inertial.stamp_sec);
        result.accepted = true;
        return result;
    }

    PlanarPoseUpdateResult updatePose(const PlanarPoseMeasurement& pose) {
        if (state_.initialized && pose2_inertial_eskf_detail::validPoseMeasurement(pose) &&
            pose.stamp_sec + kMinDt < state_.last_inertial_stamp_sec) {
            PlanarPoseUpdateResult result;
            result.time_alignment_rejected = true;
            result.reject_reason = PoseFusionRejectReason::kTimeAlignment;
            vrpn_health_.recordRejected();
            stampResultHealth(result);
            return result;
        }
        return updatePoseAtCurrentState(pose);
    }

    const PlanarRigidBodyState& state() const { return state_; }
    const ErrorCovariance& covariance() const { return covariance_; }
    bool initialized() const { return state_.initialized; }
    bool hasCorrectedBodyPose() const { return has_corrected_body_pose_; }
    const Pose2& correctedBodyPose() const { return corrected_body_pose_; }
    bool hasRawProjectedBodyPose() const { return has_raw_projected_body_pose_; }
    const Pose2& rawProjectedBodyPose() const { return raw_projected_body_pose_; }
    VrpnObservationState vrpnObservationState() const { return vrpn_health_.state(); }
    FilterHealth filterHealth() const {
        if (!state_.initialized) {
            return FilterHealth::kLost;
        }
        switch (vrpn_health_.state()) {
            case VrpnObservationState::kTrusted:
                return FilterHealth::kNominal;
            case VrpnObservationState::kSuspected:
            case VrpnObservationState::kRecovery:
                return FilterHealth::kDegraded;
            case VrpnObservationState::kFault:
                return FilterHealth::kImuOnly;
        }
        return FilterHealth::kLost;
    }
    double vrpnInnovationWindowChiSquare() const { return vrpn_health_.chiSquareWindowSum(); }
    double lastFusedPoseStampS() const { return last_fused_pose_stamp_sec_; }

  private:
    PlanarPoseUpdateResult updatePoseAtCurrentState(const PlanarPoseMeasurement& pose) {
        PlanarPoseUpdateResult result;
        if (!pose2_inertial_eskf_detail::validPoseMeasurement(pose)) {
            result.reject_reason = PoseFusionRejectReason::kInvalidInput;
            stampResultHealth(result);
            return result;
        }

        const Pose2 marker_world = markerPoseInWorld(pose.pose);
        const Pose2 predicted_marker = predictedMarkerPose(state_);
        const Eigen::Vector3d innovation =
            pose2_inertial_eskf_detail::normalizedInnovation(se2Error(predicted_marker, marker_world));
        result.position_innovation_norm = innovation.head<2>().norm();
        result.yaw_innovation_abs = std::fabs(innovation.z());
        if (result.position_innovation_norm > config_.innovation_position_gate_m ||
            result.yaw_innovation_abs > config_.innovation_yaw_gate_rad) {
            result.innovation_rejected = true;
            result.reject_reason = PoseFusionRejectReason::kInnovationGate;
            state_.last_pose_stamp_sec = pose.stamp_sec;
            vrpn_health_.recordRejected();
            stampResultHealth(result);
            return result;
        }

        if (!state_.initialized) {
            raw_projected_body_pose_ = bodyPoseFromMarkerPose(marker_world, config_.body_to_marker);
            has_raw_projected_body_pose_ = true;
            stampResultHealth(result);
            return result;
        }

        const MeasurementMatrix H = measurementJacobian(predicted_marker, innovation);
        const MeasurementCovariance R = measurementCovariance();
        const MeasurementCovariance S = H * covariance_ * H.transpose() + R;
        Eigen::LDLT<MeasurementCovariance> ldlt;
        ldlt.compute(S);
        if (ldlt.info() != Eigen::Success || !S.allFinite()) {
            result.reject_reason = PoseFusionRejectReason::kNumericalFailure;
            vrpn_health_.recordRejected();
            stampResultHealth(result);
            return result;
        }

        result.mahalanobis_distance = innovation.transpose() * ldlt.solve(innovation);
        if (!std::isfinite(result.mahalanobis_distance)) {
            result.reject_reason = PoseFusionRejectReason::kNumericalFailure;
            vrpn_health_.recordRejected();
            stampResultHealth(result);
            return result;
        }
        const bool vrpn_fault = vrpn_health_.state() == VrpnObservationState::kFault;
        vrpn_health_.recordAccepted(result.mahalanobis_distance);
        if (vrpn_fault) {
            result.reject_reason = PoseFusionRejectReason::kVrpnFault;
            stampResultHealth(result);
            return result;
        }

        const Eigen::Matrix<double, kErrorStateDim, 3> K =
            covariance_ * H.transpose() * ldlt.solve(MeasurementCovariance::Identity());
        const ErrorVector delta = K * innovation;
        if (!delta.allFinite()) {
            result.reject_reason = PoseFusionRejectReason::kNumericalFailure;
            vrpn_health_.recordRejected();
            stampResultHealth(result);
            return result;
        }
        injectError(delta);

        const ErrorCovariance identity = ErrorCovariance::Identity();
        covariance_ = (identity - K * H) * covariance_ * (identity - K * H).transpose() + K * R * K.transpose();
        covariance_ = 0.5 * (covariance_ + covariance_.transpose());
        state_.last_pose_stamp_sec = pose.stamp_sec;
        state_.covariance_trace = covariance_.trace();
        corrected_body_pose_ = bodyPoseFromState(state_);
        raw_projected_body_pose_ = bodyPoseFromMarkerPose(marker_world, state_.body_to_marker);
        has_corrected_body_pose_ = true;
        has_raw_projected_body_pose_ = true;
        last_fused_pose_stamp_sec_ = pose.stamp_sec;
        result.accepted = true;
        stampResultHealth(result);
        return result;
    }

    Pose2 markerPoseInWorld(const Pose2& raw_marker_pose) const {
        return compose(config_.measurement_frame_to_world, raw_marker_pose);
    }

    static Pose2 bodyPoseFromMarkerPose(const Pose2& marker_pose_world, const Pose2& body_to_marker) {
        return compose(marker_pose_world, inverse(body_to_marker));
    }

    static Pose2 predictedMarkerPose(const PlanarRigidBodyState& state) {
        Pose2 body_pose;
        body_pose.position = state.position;
        body_pose.yaw = state.yaw;
        return compose(body_pose, state.body_to_marker);
    }

    static Pose2 bodyPoseFromState(const PlanarRigidBodyState& state) {
        Pose2 body_pose;
        body_pose.position = state.position;
        body_pose.yaw = state.yaw;
        return body_pose;
    }

    MeasurementCovariance measurementCovariance() const {
        MeasurementCovariance R = MeasurementCovariance::Zero();
        R(0, 0) = config_.pose_position_noise_std * config_.pose_position_noise_std;
        R(1, 1) = config_.pose_position_noise_std * config_.pose_position_noise_std;
        R(2, 2) = config_.pose_yaw_noise_std * config_.pose_yaw_noise_std;
        return R;
    }

    MeasurementMatrix measurementJacobian(const Pose2& predicted_marker,
                                          const Eigen::Vector3d& innovation) const {
        MeasurementMatrix H;
        H.setZero();
        const Eigen::Matrix2d marker_rotation_transpose = rotationMatrix2(predicted_marker.yaw).transpose();
        const Eigen::Matrix2d extrinsic_rotation_transpose =
            rotationMatrix2(state_.body_to_marker.yaw).transpose();
        const Eigen::Matrix2d yaw_jacobian = pose2_inertial_eskf_detail::yawDerivativeMatrix();
        const Eigen::Vector2d residual_position = innovation.head<2>();

        H.block<2, 2>(0, 0) = marker_rotation_transpose;
        H.block<2, 1>(0, 4) =
            yaw_jacobian * residual_position +
            extrinsic_rotation_transpose * yaw_jacobian * state_.body_to_marker.position;
        H(2, 4) = 1.0;

        if (config_.estimate_extrinsic) {
            H.block<2, 2>(0, 8) = extrinsic_rotation_transpose;
            H.block<2, 1>(0, 10) = yaw_jacobian * residual_position;
            H(2, 10) = 1.0;
        }
        return H;
    }

    void propagateCovariance(const Eigen::Vector2d& accel_body, double yaw_for_linearization, double dt) {
        ErrorCovariance F = ErrorCovariance::Identity();
        const Eigen::Matrix2d rotation = rotationMatrix2(yaw_for_linearization);
        const Eigen::Vector2d accel_yaw_derivative =
            rotation * pose2_inertial_eskf_detail::yawDerivativeMatrix() * accel_body;

        F.block<2, 2>(0, 2) = Eigen::Matrix2d::Identity() * dt;
        F.block<2, 1>(0, 4) = 0.5 * accel_yaw_derivative * dt * dt;
        F.block<2, 2>(0, 6) = -0.5 * rotation * dt * dt;
        F.block<2, 1>(2, 4) = accel_yaw_derivative * dt;
        F.block<2, 2>(2, 6) = -rotation * dt;
        F(4, 5) = -dt;

        ErrorCovariance Q = ErrorCovariance::Zero();
        const double accel_var = config_.accel_noise_std * config_.accel_noise_std;
        const double gyro_var = config_.gyro_noise_std * config_.gyro_noise_std;
        Q.block<2, 2>(0, 0).diagonal().setConstant(0.25 * accel_var * dt * dt * dt * dt);
        Q.block<2, 2>(2, 2).diagonal().setConstant(accel_var * dt * dt);
        Q(4, 4) = gyro_var * dt * dt;
        Q(5, 5) = config_.gyro_bias_random_walk_std * config_.gyro_bias_random_walk_std * dt;
        Q.block<2, 2>(6, 6).diagonal().setConstant(config_.accel_bias_random_walk_std *
                                                   config_.accel_bias_random_walk_std * dt);
        if (config_.estimate_extrinsic) {
            Q.block<2, 2>(8, 8).diagonal().setConstant(config_.extrinsic_position_random_walk_std *
                                                       config_.extrinsic_position_random_walk_std * dt);
            Q(10, 10) = config_.extrinsic_yaw_random_walk_std * config_.extrinsic_yaw_random_walk_std * dt;
        }

        covariance_ = F * covariance_ * F.transpose() + Q;
        covariance_ = 0.5 * (covariance_ + covariance_.transpose());
    }

    void inflateCovariance(double dt) {
        ErrorCovariance Q = ErrorCovariance::Zero();
        Q.block<2, 2>(0, 0).diagonal().setConstant(config_.accel_noise_std * config_.accel_noise_std * dt * dt);
        Q.block<2, 2>(2, 2).diagonal().setConstant(config_.accel_noise_std * config_.accel_noise_std * dt);
        Q(4, 4) = config_.gyro_noise_std * config_.gyro_noise_std * dt;
        covariance_ += Q;
        covariance_ = 0.5 * (covariance_ + covariance_.transpose());
        state_.covariance_trace = covariance_.trace();
    }

    void injectError(const ErrorVector& delta) { injectError(delta, state_); }

    void injectError(const ErrorVector& delta, PlanarRigidBodyState& state) const {
        state.position += delta.segment<2>(0);
        state.velocity += delta.segment<2>(2);
        state.yaw = normalizeAngle(state.yaw + delta(4));
        state.gyro_bias_z += delta(5);
        state.accel_bias += delta.segment<2>(6);
        if (config_.estimate_extrinsic) {
            state.body_to_marker.position += delta.segment<2>(8);
            state.body_to_marker.yaw = normalizeAngle(state.body_to_marker.yaw + delta(10));
        }
    }

    void integrateInertialStep(const PlanarInertialSample& inertial, double dt) {
        const double omega_z = inertial.angular_velocity_z - state_.gyro_bias_z;
        const Eigen::Vector2d accel_body = inertial.linear_acceleration - state_.accel_bias;
        const double yaw_mid = normalizeAngle(state_.yaw + 0.5 * omega_z * dt);
        const Eigen::Matrix2d rotation = rotationMatrix2(yaw_mid);
        const Eigen::Vector2d accel_world = rotation * accel_body;

        state_.position += state_.velocity * dt + 0.5 * accel_world * dt * dt;
        state_.velocity += accel_world * dt;
        state_.yaw = normalizeAngle(state_.yaw + omega_z * dt);
        state_.yaw_rate = omega_z;
        propagateCovariance(accel_body, yaw_mid, dt);
        state_.covariance_trace = covariance_.trace();
    }

    bool propagateToStamp(const PlanarInertialSample& inertial, double target_stamp_sec) {
        if (!std::isfinite(target_stamp_sec) || target_stamp_sec <= state_.last_inertial_stamp_sec + kMinDt) {
            return false;
        }
        double remaining = target_stamp_sec - state_.last_inertial_stamp_sec;
        while (remaining > kMinDt) {
            const double dt = std::min(remaining, config_.max_propagation_dt_s);
            integrateInertialStep(inertial, dt);
            state_.last_inertial_stamp_sec += dt;
            remaining = target_stamp_sec - state_.last_inertial_stamp_sec;
        }
        state_.last_inertial_stamp_sec = target_stamp_sec;
        state_.covariance_trace = covariance_.trace();
        corrected_body_pose_ = bodyPoseFromState(state_);
        has_corrected_body_pose_ = true;
        return true;
    }

    void stampResultHealth(PlanarPoseUpdateResult& result) const {
        result.vrpn_observation_state = vrpn_health_.state();
        result.filter_health = filterHealth();
        result.innovation_window_chi_square = vrpn_health_.chiSquareWindowSum();
    }

    static constexpr double kMinDt = 1.0e-5;

    Pose2InertialEskfConfig config_{};
    PlanarRigidBodyState state_{};
    ErrorCovariance covariance_{ErrorCovariance::Identity()};
    Pose2 corrected_body_pose_{};
    Pose2 raw_projected_body_pose_{};
    bool has_corrected_body_pose_{false};
    bool has_raw_projected_body_pose_{false};
    ObservationHealthTracker vrpn_health_{};
    double last_fused_pose_stamp_sec_{0.0};
};

} // namespace xgc2_math
