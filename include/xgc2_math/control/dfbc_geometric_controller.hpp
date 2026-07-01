#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "xgc2_math/control/se3_nmpc_problem.hpp"
#include "xgc2_math/trajectory/trajectory3.hpp"

namespace xgc2_math::control {

struct DfbcGeometricConfig {
    Eigen::Vector3d position_natural_frequency{Eigen::Vector3d(1.2, 1.2, 1.5)};
    Eigen::Vector3d position_damping_ratio{Eigen::Vector3d(0.9, 0.9, 1.0)};
    double tilt_gain{4.0};
    double tilt_rate_damping{0.8};
    double yaw_gain{0.3};
    double yaw_rate_damping{0.2};
    double gravity{9.8066};
    double min_specific_thrust{0.1};
    bool use_body_rate_feedforward{true};
    bool enable_yaw_control{false};
    bool acceleration_correction_enabled{false};
    Eigen::Vector3d acceleration_correction_gain{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration_correction_limit{Eigen::Vector3d::Zero()};
    double acceleration_correction_filter_tau{0.0};
};

struct DfbcGeometricInput {
    Se3State current{};
    trajectory::FlatOutput3 reference{};
    bool has_measured_acceleration{false};
    Eigen::Vector3d measured_acceleration{Eigen::Vector3d::Zero()};
    double dt{0.01};
};

struct DfbcGeometricOutput {
    bool success{false};
    uint32_t flags{0U};
    double specific_thrust{0.0};
    Eigen::Quaterniond desired_attitude{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d body_rate_command{Eigen::Vector3d::Zero()};
    Eigen::Vector3d thrust_direction_error{Eigen::Vector3d::Zero()};
    double yaw_error{0.0};
    Eigen::Vector3d position_error{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity_error{Eigen::Vector3d::Zero()};
    Eigen::Vector3d nominal_acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d corrected_acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d measured_acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration_error{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration_correction{Eigen::Vector3d::Zero()};
    bool acceleration_correction_active{false};
};

class DfbcGeometricController final {
  public:
    explicit DfbcGeometricController(DfbcGeometricConfig config = {}) : config_(std::move(config)) {}

    void configure(const DfbcGeometricConfig& config) {
        config_ = config;
        reset();
    }

    const DfbcGeometricConfig& config() const { return config_; }

    void reset() {
        filtered_acceleration_error_.setZero();
        has_filtered_acceleration_error_ = false;
    }

    DfbcGeometricOutput compute(const DfbcGeometricInput& input) {
        DfbcGeometricOutput output;
        if (!finiteState(input.current) || !trajectory::TrajectoryValidator3::finite(input.reference)) {
            return output;
        }

        const Eigen::Quaterniond attitude = normalizedQuaternionOrIdentity(input.current.attitude);
        const Eigen::Matrix3d current_rotation = attitude.toRotationMatrix();
        const Eigen::Vector3d current_body_z = current_rotation.col(2);
        output.position_error = input.reference.position - input.current.position;
        output.velocity_error = input.reference.velocity - input.current.velocity;

        const Eigen::Vector3d kp = config_.position_natural_frequency.cwiseProduct(config_.position_natural_frequency);
        const Eigen::Vector3d kv =
            2.0 * config_.position_damping_ratio.cwiseProduct(config_.position_natural_frequency);
        const Eigen::Vector3d accel_cmd = input.reference.acceleration + kp.cwiseProduct(output.position_error) +
                                          kv.cwiseProduct(output.velocity_error);
        output.nominal_acceleration = accel_cmd;
        output.corrected_acceleration = accel_cmd;
        output.measured_acceleration = input.measured_acceleration;

        if (config_.acceleration_correction_enabled && input.has_measured_acceleration &&
            input.measured_acceleration.array().isFinite().all()) {
            const Eigen::Vector3d raw_error = accel_cmd - input.measured_acceleration;
            if (raw_error.array().isFinite().all()) {
                const double dt = std::max(0.0, input.dt);
                const double tau = std::max(0.0, config_.acceleration_correction_filter_tau);
                const double alpha = tau > 1.0e-9 ? clampScalar(dt / (tau + dt), 0.0, 1.0) : 1.0;
                if (!has_filtered_acceleration_error_) {
                    filtered_acceleration_error_.setZero();
                    has_filtered_acceleration_error_ = true;
                }
                filtered_acceleration_error_ += alpha * (raw_error - filtered_acceleration_error_);
                output.acceleration_error = filtered_acceleration_error_;
                output.acceleration_correction =
                    config_.acceleration_correction_gain.cwiseProduct(output.acceleration_error);
                output.acceleration_correction =
                    clampVector(output.acceleration_correction, config_.acceleration_correction_limit);
                output.corrected_acceleration = accel_cmd + output.acceleration_correction;
                output.acceleration_correction_active = true;
            }
        }

        const Eigen::Vector3d thrust_vector =
            output.corrected_acceleration + std::max(config_.gravity, 1.0e-6) * Eigen::Vector3d::UnitZ();
        const double thrust_norm = thrust_vector.norm();
        if (!std::isfinite(thrust_norm) || thrust_norm < config_.min_specific_thrust) {
            return output;
        }
        output.specific_thrust = thrust_norm;

        const Eigen::Vector3d desired_body_z = thrust_vector / thrust_norm;
        const Eigen::Matrix3d desired_rotation = rotationFromBodyZAndYaw(desired_body_z, input.reference.yaw);
        output.desired_attitude = Eigen::Quaterniond(desired_rotation);
        output.desired_attitude.normalize();

        const trajectory::FullStateReference3 full_reference =
            trajectory::FlatnessMapper3(config_.gravity, config_.min_specific_thrust).map(input.reference);
        output.flags = full_reference.flags;

        const Eigen::Vector3d tilt_error_world = current_body_z.cross(desired_body_z);
        output.thrust_direction_error = current_rotation.transpose() * tilt_error_world;
        output.yaw_error = projectedHeadingError(current_rotation, desired_rotation, desired_body_z);

        Eigen::Vector3d omega_ff = Eigen::Vector3d::Zero();
        if (config_.use_body_rate_feedforward && full_reference.body_rate.array().isFinite().all()) {
            omega_ff = current_rotation.transpose() * desired_rotation * full_reference.body_rate;
        }

        output.body_rate_command = omega_ff;
        output.body_rate_command.x() += config_.tilt_gain * output.thrust_direction_error.x() -
                                        config_.tilt_rate_damping * (input.current.body_rate.x() - omega_ff.x());
        output.body_rate_command.y() += config_.tilt_gain * output.thrust_direction_error.y() -
                                        config_.tilt_rate_damping * (input.current.body_rate.y() - omega_ff.y());

        if (config_.enable_yaw_control) {
            output.body_rate_command.z() = omega_ff.z() + config_.yaw_gain * output.yaw_error -
                                           config_.yaw_rate_damping * (input.current.body_rate.z() - omega_ff.z());
        } else {
            output.body_rate_command.z() = 0.0;
        }

        output.success = output.body_rate_command.array().isFinite().all() && std::isfinite(output.specific_thrust);
        return output;
    }

  private:
    static bool finiteState(const Se3State& state) {
        return state.position.array().isFinite().all() && state.velocity.array().isFinite().all() &&
               state.body_rate.array().isFinite().all() && std::isfinite(state.attitude.norm()) &&
               state.attitude.norm() > 1.0e-9;
    }

    static Eigen::Vector3d normalizeProjectedHeading(const Eigen::Vector3d& axis, const Eigen::Vector3d& normal) {
        const Eigen::Vector3d projected = axis - normal * normal.dot(axis);
        const double norm = projected.norm();
        if (!std::isfinite(norm) || norm < 1.0e-9) {
            return Eigen::Vector3d::Zero();
        }
        return projected / norm;
    }

    static double projectedHeadingError(const Eigen::Matrix3d& current_rotation,
                                        const Eigen::Matrix3d& desired_rotation,
                                        const Eigen::Vector3d& desired_body_z) {
        const Eigen::Vector3d current_x = normalizeProjectedHeading(current_rotation.col(0), desired_body_z);
        const Eigen::Vector3d desired_x = normalizeProjectedHeading(desired_rotation.col(0), desired_body_z);
        if (current_x.isZero(1.0e-9) || desired_x.isZero(1.0e-9)) {
            return 0.0;
        }
        return desired_body_z.dot(current_x.cross(desired_x));
    }

    static double clampScalar(double value, double min_value, double max_value) {
        return std::max(min_value, std::min(value, max_value));
    }

    static Eigen::Vector3d clampVector(const Eigen::Vector3d& value, const Eigen::Vector3d& limit) {
        Eigen::Vector3d clamped = value;
        for (int i = 0; i < 3; ++i) {
            const double bound = std::max(0.0, limit[i]);
            clamped[i] = clampScalar(clamped[i], -bound, bound);
        }
        return clamped;
    }

    DfbcGeometricConfig config_{};
    Eigen::Vector3d filtered_acceleration_error_{Eigen::Vector3d::Zero()};
    bool has_filtered_acceleration_error_{false};
};

} // namespace xgc2_math::control
