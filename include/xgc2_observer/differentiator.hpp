#ifndef XGC2_OBSERVER_DIFFERENTIATOR_HPP
#define XGC2_OBSERVER_DIFFERENTIATOR_HPP

#include <algorithm>
#include <cmath>
#include <limits>

#include "xgc2_observer/butterworth_filter.hpp"
#include "xgc2_observer/status.hpp"

namespace xgc2_observer {

struct DifferentiatorOptions {
    double min_dt_s{1.0e-4};
    double max_dt_s{0.5};
    double max_input_step{std::numeric_limits<double>::infinity()};
    double max_derivative{std::numeric_limits<double>::infinity()};
    double derivative_cutoff_hz{0.0};
    bool reset_on_large_dt{false};
};

inline bool isValid(const DifferentiatorOptions& options)
{
    return std::isfinite(options.min_dt_s) && options.min_dt_s > 0.0 &&
           std::isfinite(options.max_dt_s) && options.max_dt_s >= options.min_dt_s &&
           (!std::isfinite(options.max_input_step) || options.max_input_step >= 0.0) &&
           (!std::isfinite(options.max_derivative) || options.max_derivative >= 0.0) &&
           std::isfinite(options.derivative_cutoff_hz) && options.derivative_cutoff_hz >= 0.0;
}

inline DifferentiatorOptions normalized(DifferentiatorOptions options)
{
    const DifferentiatorOptions defaults;
    if (!std::isfinite(options.min_dt_s) || options.min_dt_s <= 0.0) {
        options.min_dt_s = defaults.min_dt_s;
    }
    if (!std::isfinite(options.max_dt_s) || options.max_dt_s < options.min_dt_s) {
        options.max_dt_s = std::max(defaults.max_dt_s, options.min_dt_s);
    }
    if (std::isfinite(options.max_input_step) && options.max_input_step < 0.0) {
        options.max_input_step = defaults.max_input_step;
    }
    if (std::isfinite(options.max_derivative) && options.max_derivative < 0.0) {
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
    SampleStatus status{SampleStatus::kInitialized};
    bool measurement_accepted{false};
};

class Differentiator {
public:
    Differentiator() = default;

    explicit Differentiator(DifferentiatorOptions options)
        : options_(normalized(options))
    {
    }

    void setOptions(DifferentiatorOptions options)
    {
        options_ = normalized(options);
        derivative_filter_.reset(options_.derivative_cutoff_hz, derivative_);
    }

    const DifferentiatorOptions& options() const
    {
        return options_;
    }

    void reset()
    {
        initialized_ = false;
        value_ = 0.0;
        derivative_ = 0.0;
        derivative_filter_.reset(options_.derivative_cutoff_hz, 0.0);
    }

    void reset(double value, double derivative = 0.0)
    {
        initialized_ = true;
        value_ = value;
        derivative_ = derivative;
        derivative_filter_.reset(options_.derivative_cutoff_hz, derivative);
    }

    DifferentiatorSample update(double value, double dt_s)
    {
        if (!std::isfinite(value)) {
            return sample(SampleStatus::kHeldInvalidInput, false);
        }

        if (!initialized_) {
            reset(value, 0.0);
            return sample(SampleStatus::kInitialized, true);
        }

        if (!validDt(dt_s)) {
            if (options_.reset_on_large_dt && std::isfinite(dt_s) && dt_s > options_.max_dt_s) {
                reset(value, 0.0);
                return sample(SampleStatus::kInitialized, true);
            }
            return sample(SampleStatus::kHeldInvalidDt, false);
        }

        const double delta = value - value_;
        if (std::fabs(delta) > options_.max_input_step) {
            return sample(SampleStatus::kHeldOutlier, false);
        }

        const double raw_derivative = delta / dt_s;
        if (std::fabs(raw_derivative) > options_.max_derivative) {
            return sample(SampleStatus::kHeldOutlier, false);
        }

        value_ = value;
        derivative_ = options_.derivative_cutoff_hz > 0.0
                          ? derivative_filter_.filter(raw_derivative, dt_s)
                          : raw_derivative;
        return sample(SampleStatus::kAccepted, true);
    }

    double value() const
    {
        return value_;
    }

    double derivative() const
    {
        return derivative_;
    }

    bool initialized() const
    {
        return initialized_;
    }

private:
    bool validDt(double dt_s) const
    {
        return std::isfinite(dt_s) && dt_s >= options_.min_dt_s && dt_s <= options_.max_dt_s;
    }

    DifferentiatorSample sample(SampleStatus status, bool accepted) const
    {
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

}  // namespace xgc2_observer

#endif  // XGC2_OBSERVER_DIFFERENTIATOR_HPP
