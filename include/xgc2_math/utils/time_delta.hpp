#ifndef XGC2_MATH_TIME_DELTA_HPP
#define XGC2_MATH_TIME_DELTA_HPP

#include <algorithm>
#include <array>
#include <cmath>

#include "xgc2_math/utils/status.hpp"

namespace xgc2_math {

enum class TimeDeltaStatus {
    kInitialized,
    kAccepted,
    kHeldInvalidInput,
    kHeldInvalidDt,
    kHeldTimeWentBack,
};

template <> struct StatusRegistry<TimeDeltaStatus> {
    static constexpr std::array<StatusDescriptor<TimeDeltaStatus>, 5> statuses{{
        {TimeDeltaStatus::kInitialized, "initialized"},
        {TimeDeltaStatus::kAccepted, "accepted"},
        {TimeDeltaStatus::kHeldInvalidInput, "held_invalid_input"},
        {TimeDeltaStatus::kHeldInvalidDt, "held_invalid_dt"},
        {TimeDeltaStatus::kHeldTimeWentBack, "held_time_went_back"},
    }};
};

struct TimeDeltaGuardOptions {
    double min_dt_s{1.0e-4};
    double max_dt_s{0.5};
    bool reset_on_large_dt{true};
    bool reset_on_time_jump_back{true};
};

inline bool isValid(const TimeDeltaGuardOptions& options) {
    return std::isfinite(options.min_dt_s) && options.min_dt_s > 0.0 && std::isfinite(options.max_dt_s) &&
           options.max_dt_s >= options.min_dt_s;
}

inline TimeDeltaGuardOptions normalized(TimeDeltaGuardOptions options) {
    const TimeDeltaGuardOptions defaults;
    if (!std::isfinite(options.min_dt_s) || options.min_dt_s <= 0.0) {
        options.min_dt_s = defaults.min_dt_s;
    }
    if (!std::isfinite(options.max_dt_s) || options.max_dt_s < options.min_dt_s) {
        options.max_dt_s = std::max(defaults.max_dt_s, options.min_dt_s);
    }
    return options;
}

struct TimeDeltaSample {
    double dt_s{0.0};
    double timestamp_s{0.0};
    TimeDeltaStatus status{TimeDeltaStatus::kInitialized};
    bool accepted{false};
};

class TimeDeltaGuard {
  public:
    TimeDeltaGuard() = default;

    explicit TimeDeltaGuard(const TimeDeltaGuardOptions& options) : options_(normalized(options)) {}

    void setOptions(const TimeDeltaGuardOptions& options) { options_ = normalized(options); }

    const TimeDeltaGuardOptions& options() const { return options_; }

    void reset() {
        initialized_ = false;
        last_timestamp_s_ = 0.0;
    }

    void reset(double timestamp_s) {
        initialized_ = std::isfinite(timestamp_s);
        last_timestamp_s_ = initialized_ ? timestamp_s : 0.0;
    }

    TimeDeltaSample update(double timestamp_s) {
        if (!std::isfinite(timestamp_s)) {
            return sample(0.0, timestamp_s, TimeDeltaStatus::kHeldInvalidInput, false);
        }

        if (!initialized_) {
            reset(timestamp_s);
            return sample(0.0, timestamp_s, TimeDeltaStatus::kInitialized, false);
        }

        const double dt_s = timestamp_s - last_timestamp_s_;
        if (dt_s < 0.0) {
            if (options_.reset_on_time_jump_back) {
                reset(timestamp_s);
            }
            return sample(0.0, timestamp_s, TimeDeltaStatus::kHeldTimeWentBack, false);
        }

        if (dt_s < options_.min_dt_s || dt_s > options_.max_dt_s) {
            if (options_.reset_on_large_dt && dt_s > options_.max_dt_s) {
                reset(timestamp_s);
            }
            return sample(dt_s, timestamp_s, TimeDeltaStatus::kHeldInvalidDt, false);
        }

        last_timestamp_s_ = timestamp_s;
        return sample(dt_s, timestamp_s, TimeDeltaStatus::kAccepted, true);
    }

    double lastTimestampS() const { return last_timestamp_s_; }

    bool initialized() const { return initialized_; }

  private:
    static TimeDeltaSample sample(double dt_s, double timestamp_s, TimeDeltaStatus status, bool accepted) {
        TimeDeltaSample output;
        output.dt_s = dt_s;
        output.timestamp_s = timestamp_s;
        output.status = status;
        output.accepted = accepted;
        return output;
    }

    TimeDeltaGuardOptions options_{};
    double last_timestamp_s_{0.0};
    bool initialized_{false};
};

} // namespace xgc2_math

#endif // XGC2_MATH_TIME_DELTA_HPP
