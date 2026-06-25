#ifndef XGC2_MATH_LUENBERGER_OBSERVER_HPP
#define XGC2_MATH_LUENBERGER_OBSERVER_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "xgc2_math/algebra/angle.hpp"
#include "xgc2_math/observer/differentiator.hpp"

namespace xgc2_math {

enum class PositionVelocityObserverStatus {
    kInitialized,
    kAccepted,
    kHeldInvalidInput,
    kHeldInvalidDt,
    kHeldOutlier,
};

template <> struct StatusRegistry<PositionVelocityObserverStatus> {
    static constexpr std::array<StatusDescriptor<PositionVelocityObserverStatus>, 5> statuses{{
        {PositionVelocityObserverStatus::kInitialized, "initialized"},
        {PositionVelocityObserverStatus::kAccepted, "accepted"},
        {PositionVelocityObserverStatus::kHeldInvalidInput, "held_invalid_input"},
        {PositionVelocityObserverStatus::kHeldInvalidDt, "held_invalid_dt"},
        {PositionVelocityObserverStatus::kHeldOutlier, "held_outlier"},
    }};
};

struct PositionVelocityObserverOptions {
    double position_gain{0.35};
    double velocity_gain{0.08};
    double min_dt_s{1.0e-4};
    double max_dt_s{0.5};
    double max_position_residual{std::numeric_limits<double>::infinity()};
    double max_velocity{std::numeric_limits<double>::infinity()};
};

inline bool isValid(const PositionVelocityObserverOptions& options) {
    return std::isfinite(options.position_gain) && options.position_gain >= 0.0 &&
           std::isfinite(options.velocity_gain) && options.velocity_gain >= 0.0 && std::isfinite(options.min_dt_s) &&
           options.min_dt_s > 0.0 && std::isfinite(options.max_dt_s) && options.max_dt_s >= options.min_dt_s &&
           isNonnegativeLimit(options.max_position_residual) && isNonnegativeLimit(options.max_velocity);
}

inline PositionVelocityObserverOptions normalized(PositionVelocityObserverOptions options) {
    const PositionVelocityObserverOptions defaults;
    if (!std::isfinite(options.position_gain) || options.position_gain < 0.0) {
        options.position_gain = defaults.position_gain;
    }
    if (!std::isfinite(options.velocity_gain) || options.velocity_gain < 0.0) {
        options.velocity_gain = defaults.velocity_gain;
    }
    if (!std::isfinite(options.min_dt_s) || options.min_dt_s <= 0.0) {
        options.min_dt_s = defaults.min_dt_s;
    }
    if (!std::isfinite(options.max_dt_s) || options.max_dt_s < options.min_dt_s) {
        options.max_dt_s = std::max(defaults.max_dt_s, options.min_dt_s);
    }
    if (!isNonnegativeLimit(options.max_position_residual)) {
        options.max_position_residual = defaults.max_position_residual;
    }
    if (!isNonnegativeLimit(options.max_velocity)) {
        options.max_velocity = defaults.max_velocity;
    }
    return options;
}

struct PositionVelocityEstimate {
    double position{0.0};
    double velocity{0.0};
    double residual{0.0};
    PositionVelocityObserverStatus status{PositionVelocityObserverStatus::kInitialized};
    bool measurement_accepted{false};
};

class PositionVelocityLuenbergerObserver {
  public:
    PositionVelocityLuenbergerObserver() = default;

    explicit PositionVelocityLuenbergerObserver(const PositionVelocityObserverOptions& options)
        : options_(normalized(options)) {}

    void setOptions(const PositionVelocityObserverOptions& options) { options_ = normalized(options); }

    const PositionVelocityObserverOptions& options() const { return options_; }

    void reset() {
        initialized_ = false;
        position_ = 0.0;
        velocity_ = 0.0;
    }

    void reset(double position, double velocity = 0.0) {
        initialized_ = true;
        position_ = position;
        velocity_ = velocity;
    }

    PositionVelocityEstimate update(double measured_position, double dt_s, double acceleration = 0.0) {
        if (!std::isfinite(measured_position) || !std::isfinite(acceleration)) {
            return estimate(0.0, PositionVelocityObserverStatus::kHeldInvalidInput, false);
        }

        if (!initialized_) {
            reset(measured_position, 0.0);
            return estimate(0.0, PositionVelocityObserverStatus::kInitialized, true);
        }

        if (!validDt(dt_s)) {
            return estimate(0.0, PositionVelocityObserverStatus::kHeldInvalidDt, false);
        }

        const double predicted_position = position_ + velocity_ * dt_s + 0.5 * acceleration * dt_s * dt_s;
        const double predicted_velocity = velocity_ + acceleration * dt_s;
        const double residual = measured_position - predicted_position;

        position_ = predicted_position;
        velocity_ = predicted_velocity;

        if (std::fabs(residual) > options_.max_position_residual) {
            clampVelocity();
            return estimate(residual, PositionVelocityObserverStatus::kHeldOutlier, false);
        }

        position_ += options_.position_gain * residual;
        velocity_ += options_.velocity_gain * residual / dt_s;
        clampVelocity();
        return estimate(residual, PositionVelocityObserverStatus::kAccepted, true);
    }

    double position() const { return position_; }

    double velocity() const { return velocity_; }

    bool initialized() const { return initialized_; }

  private:
    bool validDt(double dt_s) const {
        return std::isfinite(dt_s) && dt_s >= options_.min_dt_s && dt_s <= options_.max_dt_s;
    }

    void clampVelocity() {
        if (std::fabs(velocity_) > options_.max_velocity) {
            velocity_ = std::copysign(options_.max_velocity, velocity_);
        }
    }

    PositionVelocityEstimate estimate(double residual, PositionVelocityObserverStatus status, bool accepted) const {
        PositionVelocityEstimate output;
        output.position = position_;
        output.velocity = velocity_;
        output.residual = residual;
        output.status = status;
        output.measurement_accepted = accepted;
        return output;
    }

    PositionVelocityObserverOptions options_{};
    double position_{0.0};
    double velocity_{0.0};
    bool initialized_{false};
};

class AngularPositionVelocityLuenbergerObserver {
  public:
    AngularPositionVelocityLuenbergerObserver() = default;

    explicit AngularPositionVelocityLuenbergerObserver(const PositionVelocityObserverOptions& options)
        : options_(normalized(options)) {}

    void setOptions(const PositionVelocityObserverOptions& options) { options_ = normalized(options); }

    const PositionVelocityObserverOptions& options() const { return options_; }

    void reset() {
        initialized_ = false;
        angle_rad_ = 0.0;
        angular_velocity_rad_s_ = 0.0;
    }

    void reset(double angle_rad, double angular_velocity_rad_s = 0.0) {
        initialized_ = true;
        angle_rad_ = normalizeAngle(angle_rad);
        angular_velocity_rad_s_ = angular_velocity_rad_s;
    }

    PositionVelocityEstimate update(double measured_angle_rad, double dt_s, double angular_acceleration_rad_s2 = 0.0) {
        if (!std::isfinite(measured_angle_rad) || !std::isfinite(angular_acceleration_rad_s2)) {
            return estimate(0.0, PositionVelocityObserverStatus::kHeldInvalidInput, false);
        }

        const double normalized_measurement = normalizeAngle(measured_angle_rad);
        if (!initialized_) {
            reset(normalized_measurement, 0.0);
            return estimate(0.0, PositionVelocityObserverStatus::kInitialized, true);
        }

        if (!validDt(dt_s)) {
            return estimate(0.0, PositionVelocityObserverStatus::kHeldInvalidDt, false);
        }

        const double predicted_angle = normalizeAngle(angle_rad_ + angular_velocity_rad_s_ * dt_s +
                                                      0.5 * angular_acceleration_rad_s2 * dt_s * dt_s);
        const double predicted_velocity = angular_velocity_rad_s_ + angular_acceleration_rad_s2 * dt_s;
        const double residual = shortestAngularDistance(predicted_angle, normalized_measurement);

        angle_rad_ = predicted_angle;
        angular_velocity_rad_s_ = predicted_velocity;

        if (std::fabs(residual) > options_.max_position_residual) {
            clampVelocity();
            return estimate(residual, PositionVelocityObserverStatus::kHeldOutlier, false);
        }

        angle_rad_ = normalizeAngle(angle_rad_ + options_.position_gain * residual);
        angular_velocity_rad_s_ += options_.velocity_gain * residual / dt_s;
        clampVelocity();
        return estimate(residual, PositionVelocityObserverStatus::kAccepted, true);
    }

    double position() const { return angle_rad_; }

    double velocity() const { return angular_velocity_rad_s_; }

    bool initialized() const { return initialized_; }

  private:
    bool validDt(double dt_s) const {
        return std::isfinite(dt_s) && dt_s >= options_.min_dt_s && dt_s <= options_.max_dt_s;
    }

    void clampVelocity() {
        if (std::fabs(angular_velocity_rad_s_) > options_.max_velocity) {
            angular_velocity_rad_s_ = std::copysign(options_.max_velocity, angular_velocity_rad_s_);
        }
    }

    PositionVelocityEstimate estimate(double residual, PositionVelocityObserverStatus status, bool accepted) const {
        PositionVelocityEstimate output;
        output.position = angle_rad_;
        output.velocity = angular_velocity_rad_s_;
        output.residual = residual;
        output.status = status;
        output.measurement_accepted = accepted;
        return output;
    }

    PositionVelocityObserverOptions options_{};
    double angle_rad_{0.0};
    double angular_velocity_rad_s_{0.0};
    bool initialized_{false};
};

} // namespace xgc2_math

#endif // XGC2_MATH_LUENBERGER_OBSERVER_HPP
