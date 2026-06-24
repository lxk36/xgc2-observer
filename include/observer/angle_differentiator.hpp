#ifndef XGC2_MATH_ANGLE_DIFFERENTIATOR_HPP
#define XGC2_MATH_ANGLE_DIFFERENTIATOR_HPP

#include <cmath>

#include "core/angle.hpp"
#include "observer/differentiator.hpp"

namespace xgc2_math {

class AngleDifferentiator {
  public:
    AngleDifferentiator() = default;

    explicit AngleDifferentiator(const DifferentiatorOptions& options) : options_(normalized(options)) {}

    void setOptions(const DifferentiatorOptions& options) {
        options_ = normalized(options);
        derivative_filter_.reset(options_.derivative_cutoff_hz, derivative_);
    }

    const DifferentiatorOptions& options() const { return options_; }

    void reset() {
        initialized_ = false;
        angle_rad_ = 0.0;
        derivative_ = 0.0;
        derivative_filter_.reset(options_.derivative_cutoff_hz, 0.0);
    }

    void reset(double angle_rad, double derivative = 0.0) {
        initialized_ = true;
        angle_rad_ = normalizeAngle(angle_rad);
        derivative_ = derivative;
        derivative_filter_.reset(options_.derivative_cutoff_hz, derivative);
    }

    DifferentiatorSample update(double angle_rad, double dt_s) {
        if (!std::isfinite(angle_rad)) {
            return sample(SampleStatus::kHeldInvalidInput, false);
        }

        const double normalized_angle = normalizeAngle(angle_rad);
        if (!initialized_) {
            reset(normalized_angle, 0.0);
            return sample(SampleStatus::kInitialized, true);
        }

        if (!validDt(dt_s)) {
            if (options_.reset_on_large_dt && std::isfinite(dt_s) && dt_s > options_.max_dt_s) {
                reset(normalized_angle, 0.0);
                return sample(SampleStatus::kInitialized, true);
            }
            return sample(SampleStatus::kHeldInvalidDt, false);
        }

        const double delta = shortestAngularDistance(angle_rad_, normalized_angle);
        if (std::fabs(delta) > options_.max_input_step) {
            return sample(SampleStatus::kHeldOutlier, false);
        }

        const double raw_derivative = delta / dt_s;
        if (std::fabs(raw_derivative) > options_.max_derivative) {
            return sample(SampleStatus::kHeldOutlier, false);
        }

        angle_rad_ = normalized_angle;
        derivative_ =
            options_.derivative_cutoff_hz > 0.0 ? derivative_filter_.filter(raw_derivative, dt_s) : raw_derivative;
        return sample(SampleStatus::kAccepted, true);
    }

    double value() const { return angle_rad_; }

    double derivative() const { return derivative_; }

    bool initialized() const { return initialized_; }

  private:
    bool validDt(double dt_s) const {
        return std::isfinite(dt_s) && dt_s >= options_.min_dt_s && dt_s <= options_.max_dt_s;
    }

    DifferentiatorSample sample(SampleStatus status, bool accepted) const {
        DifferentiatorSample output;
        output.value = angle_rad_;
        output.derivative = derivative_;
        output.status = status;
        output.measurement_accepted = accepted;
        return output;
    }

    DifferentiatorOptions options_{};
    SecondOrderButterworthLowPass derivative_filter_{};
    double angle_rad_{0.0};
    double derivative_{0.0};
    bool initialized_{false};
};

} // namespace xgc2_math

#endif // XGC2_MATH_ANGLE_DIFFERENTIATOR_HPP
