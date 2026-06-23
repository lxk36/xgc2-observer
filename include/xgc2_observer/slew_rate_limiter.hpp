#ifndef XGC2_OBSERVER_SLEW_RATE_LIMITER_HPP
#define XGC2_OBSERVER_SLEW_RATE_LIMITER_HPP

#include <algorithm>
#include <cmath>

namespace xgc2_observer {

class SlewRateLimiter {
  public:
    SlewRateLimiter() = default;

    explicit SlewRateLimiter(double max_rate_per_second, double initial_value = 0.0) {
        reset(max_rate_per_second, initial_value);
    }

    void reset(double max_rate_per_second, double initial_value = 0.0) {
        setMaxRatePerSecond(max_rate_per_second);
        resetState(initial_value);
    }

    void setMaxRatePerSecond(double max_rate_per_second) {
        max_rate_per_second_ =
            std::isfinite(max_rate_per_second) ? std::max(0.0, max_rate_per_second) : 0.0;
    }

    void resetState(double value = 0.0) {
        value_ = value;
        initialized_ = true;
    }

    double filter(double input, double dt_s) {
        if (!std::isfinite(input)) {
            return value_;
        }
        if (!initialized_) {
            resetState(input);
            return input;
        }
        if (max_rate_per_second_ <= 0.0 || !std::isfinite(dt_s) || dt_s <= 0.0) {
            resetState(input);
            return input;
        }

        const double max_delta = max_rate_per_second_ * dt_s;
        value_ += std::clamp(input - value_, -max_delta, max_delta);
        return value_;
    }

    double value() const { return value_; }

    double maxRatePerSecond() const { return max_rate_per_second_; }

    bool initialized() const { return initialized_; }

  private:
    double max_rate_per_second_{0.0};
    double value_{0.0};
    bool initialized_{false};
};

} // namespace xgc2_observer

#endif // XGC2_OBSERVER_SLEW_RATE_LIMITER_HPP
