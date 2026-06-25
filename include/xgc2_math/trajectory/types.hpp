#pragma once

#include <cstdint>

namespace xgc2_math::trajectory {

enum class TrajectoryModelType : uint8_t {
    kNone = 0,
    kAnalytic = 1,
    kPolynomial = 2,
    kSampled = 3,
};

enum TrajectoryFlag : uint32_t {
    kFlagNone = 0U,
    kFlagInvalidInput = 1U << 0U,
    kFlagTimeDomain = 1U << 1U,
    kFlagNonFinite = 1U << 2U,
    kFlagLowThrust = 1U << 3U,
    kFlagYawSingularity = 1U << 4U,
    kFlagVelocityLimit = 1U << 5U,
    kFlagAccelerationLimit = 1U << 6U,
    kFlagJerkLimit = 1U << 7U,
    kFlagSnapLimit = 1U << 8U,
    kFlagBodyRateLimit = 1U << 9U,
    kFlagTiltLimit = 1U << 10U,
    kFlagThrustLimit = 1U << 11U,
    kFlagOptimizationFailure = 1U << 12U,
    kFlagLowSpeedSingularity = 1U << 13U,
    kFlagYawRateLimit = 1U << 14U,
};

struct TrajectoryValidationResult {
    uint32_t flags{kFlagNone};

    bool ok() const {
        return (flags & (kFlagInvalidInput | kFlagTimeDomain | kFlagNonFinite |
                         kFlagLowThrust | kFlagYawSingularity |
                         kFlagOptimizationFailure | kFlagLowSpeedSingularity)) == 0U;
    }
};

}  // namespace xgc2_math::trajectory

