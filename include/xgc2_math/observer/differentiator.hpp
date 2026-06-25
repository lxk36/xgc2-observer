#ifndef XGC2_MATH_DIFFERENTIATOR_HPP
#define XGC2_MATH_DIFFERENTIATOR_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "xgc2_math/utils/status.hpp"
#include "xgc2_math/filter/butterworth_filter.hpp"

namespace xgc2_math {

enum class DifferentiatorStatus {
    kInitialized,
    kAccepted,
    kHeldInvalidInput,
    kHeldInvalidDt,
    kHeldOutlier,
};

template <> struct StatusRegistry<DifferentiatorStatus> {
    static constexpr std::array<StatusDescriptor<DifferentiatorStatus>, 5> statuses{{
        {DifferentiatorStatus::kInitialized, "initialized"},
        {DifferentiatorStatus::kAccepted, "accepted"},
        {DifferentiatorStatus::kHeldInvalidInput, "held_invalid_input"},
        {DifferentiatorStatus::kHeldInvalidDt, "held_invalid_dt"},
        {DifferentiatorStatus::kHeldOutlier, "held_outlier"},
    }};
};

inline bool isNonnegativeLimit(double value) {
    return std::isinf(value) || (std::isfinite(value) && value >= 0.0);
}

struct DifferentiatorOptions {
    double min_dt_s{1.0e-4};
    double max_dt_s{0.5};
    double max_input_step{std::numeric_limits<double>::infinity()};
    double max_derivative{std::numeric_limits<double>::infinity()};
    double derivative_cutoff_hz{0.0};
    bool reset_on_large_dt{false};
};

inline bool isValid(const DifferentiatorOptions& options) {
    return std::isfinite(options.min_dt_s) && options.min_dt_s > 0.0 && std::isfinite(options.max_dt_s) &&
           options.max_dt_s >= options.min_dt_s && isNonnegativeLimit(options.max_input_step) &&
           isNonnegativeLimit(options.max_derivative) && std::isfinite(options.derivative_cutoff_hz) &&
           options.derivative_cutoff_hz >= 0.0;
}

inline DifferentiatorOptions normalized(DifferentiatorOptions options) {
    const DifferentiatorOptions defaults;
    if (!std::isfinite(options.min_dt_s) || options.min_dt_s <= 0.0) {
        options.min_dt_s = defaults.min_dt_s;
    }
    if (!std::isfinite(options.max_dt_s) || options.max_dt_s < options.min_dt_s) {
        options.max_dt_s = std::max(defaults.max_dt_s, options.min_dt_s);
    }
    if (!isNonnegativeLimit(options.max_input_step)) {
        options.max_input_step = defaults.max_input_step;
    }
    if (!isNonnegativeLimit(options.max_derivative)) {
        options.max_derivative = defaults.max_derivative;
    }
    if (!std::isfinite(options.derivative_cutoff_hz) || options.derivative_cutoff_hz < 0.0) {
        options.derivative_cutoff_hz = defaults.derivative_cutoff_hz;
    }
    return options;
}

struct DifferentiatorSample {
    double value{0.0};
    double derivative{0.0};
    DifferentiatorStatus status{DifferentiatorStatus::kInitialized};
    bool measurement_accepted{false};
};

class Differentiator {
  public:
    Differentiator() = default;

    explicit Differentiator(const DifferentiatorOptions& options) : options_(normalized(options)) {}

    void setOptions(const DifferentiatorOptions& options) {
        options_ = normalized(options);
        derivative_filter_.reset(options_.derivative_cutoff_hz, derivative_);
    }

    const DifferentiatorOptions& options() const { return options_; }

    void reset() {
        initialized_ = false;
        value_ = 0.0;
        derivative_ = 0.0;
        derivative_filter_.reset(options_.derivative_cutoff_hz, 0.0);
    }

    void reset(double value, double derivative = 0.0) {
        initialized_ = true;
        value_ = value;
        derivative_ = derivative;
        derivative_filter_.reset(options_.derivative_cutoff_hz, derivative);
    }

    DifferentiatorSample update(double value, double dt_s) {
        if (!std::isfinite(value)) {
            return sample(DifferentiatorStatus::kHeldInvalidInput, false);
        }

        if (!initialized_) {
            reset(value, 0.0);
            return sample(DifferentiatorStatus::kInitialized, true);
        }

        if (!validDt(dt_s)) {
            if (options_.reset_on_large_dt && std::isfinite(dt_s) && dt_s > options_.max_dt_s) {
                reset(value, 0.0);
                return sample(DifferentiatorStatus::kInitialized, true);
            }
            return sample(DifferentiatorStatus::kHeldInvalidDt, false);
        }

        const double delta = value - value_;
        if (std::fabs(delta) > options_.max_input_step) {
            return sample(DifferentiatorStatus::kHeldOutlier, false);
        }

        const double raw_derivative = delta / dt_s;
        if (std::fabs(raw_derivative) > options_.max_derivative) {
            return sample(DifferentiatorStatus::kHeldOutlier, false);
        }

        value_ = value;
        derivative_ =
            options_.derivative_cutoff_hz > 0.0 ? derivative_filter_.filter(raw_derivative, dt_s) : raw_derivative;
        return sample(DifferentiatorStatus::kAccepted, true);
    }

    double value() const { return value_; }

    double derivative() const { return derivative_; }

    bool initialized() const { return initialized_; }

  private:
    bool validDt(double dt_s) const {
        return std::isfinite(dt_s) && dt_s >= options_.min_dt_s && dt_s <= options_.max_dt_s;
    }

    DifferentiatorSample sample(DifferentiatorStatus status, bool accepted) const {
        DifferentiatorSample output;
        output.value = value_;
        output.derivative = derivative_;
        output.status = status;
        output.measurement_accepted = accepted;
        return output;
    }

    DifferentiatorOptions options_{};
    SecondOrderButterworthLowPass derivative_filter_{};
    double value_{0.0};
    double derivative_{0.0};
    bool initialized_{false};
};

} // namespace xgc2_math

#endif // XGC2_MATH_DIFFERENTIATOR_HPP
