#ifndef XGC2_MATH_ALGEBRA_ANGLE_HPP
#define XGC2_MATH_ALGEBRA_ANGLE_HPP

#include <cmath>
#include <limits>

namespace xgc2_math {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

inline double normalizeAngle(double angle_rad) {
    if (!std::isfinite(angle_rad)) {
        return angle_rad;
    }
    double result = std::fmod(angle_rad + kPi, kTwoPi);
    if (result < 0.0) {
        result += kTwoPi;
    }
    result -= kPi;
    if (result <= -kPi) {
        result += kTwoPi;
    }
    return result;
}

inline double shortestAngularDistance(double from_rad, double to_rad) {
    if (!std::isfinite(from_rad) || !std::isfinite(to_rad)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return normalizeAngle(to_rad - from_rad);
}

} // namespace xgc2_math

#endif // XGC2_MATH_ALGEBRA_ANGLE_HPP
