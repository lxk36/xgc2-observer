#ifndef XGC2_OBSERVER_STATUS_HPP
#define XGC2_OBSERVER_STATUS_HPP

namespace xgc2_observer {

enum class SampleStatus {
    kInitialized,
    kAccepted,
    kHeldInvalidInput,
    kHeldInvalidDt,
    kHeldOutlier,
    kHeldTimeWentBack,
};

inline const char* toString(SampleStatus status)
{
    switch (status) {
    case SampleStatus::kInitialized:
        return "initialized";
    case SampleStatus::kAccepted:
        return "accepted";
    case SampleStatus::kHeldInvalidInput:
        return "held_invalid_input";
    case SampleStatus::kHeldInvalidDt:
        return "held_invalid_dt";
    case SampleStatus::kHeldOutlier:
        return "held_outlier";
    case SampleStatus::kHeldTimeWentBack:
        return "held_time_went_back";
    }
    return "unknown";
}

inline bool measurementAccepted(SampleStatus status)
{
    return status == SampleStatus::kInitialized || status == SampleStatus::kAccepted;
}

inline bool measurementHeld(SampleStatus status)
{
    return !measurementAccepted(status);
}

}  // namespace xgc2_observer

#endif  // XGC2_OBSERVER_STATUS_HPP
