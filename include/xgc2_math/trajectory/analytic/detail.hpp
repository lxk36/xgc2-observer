#pragma once

#include "xgc2_math/trajectory/trajectory2.hpp"
#include "xgc2_math/trajectory/trajectory3.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <vector>

namespace xgc2_math::trajectory::analytic_detail {

constexpr double kMinDuration = 1.0e-6;

inline bool finiteScalar(double value) {
    return std::isfinite(value);
}

inline double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(max_value, value));
}

inline double safeRadius(double radius) {
    return std::max(1.0e-3, std::abs(radius));
}

inline double angularRate(double radius, double line_speed) {
    return line_speed / safeRadius(radius);
}

inline double polyValue(const std::vector<double>& coeffs, double t, int derivative) {
    if (derivative < 0 || coeffs.empty() || derivative >= static_cast<int>(coeffs.size())) {
        return 0.0;
    }
    double value = 0.0;
    for (int i = static_cast<int>(coeffs.size()) - 1; i >= derivative; --i) {
        double factor = 1.0;
        for (int k = 0; k < derivative; ++k) {
            factor *= static_cast<double>(i - k);
        }
        value = value * t + factor * coeffs[static_cast<size_t>(i)];
    }
    return value;
}

inline std::vector<double> septicBoundary(double p0, double v0, double a0, double j0, double p1,
                                          double v1, double a1, double j1, double duration) {
    const double T = std::max(kMinDuration, duration);
    std::vector<double> c(8U, 0.0);
    c[0] = p0;
    c[1] = v0;
    c[2] = 0.5 * a0;
    c[3] = j0 / 6.0;

    Eigen::Matrix4d A;
    Eigen::Vector4d b;
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T3 * T;
    const double T5 = T4 * T;
    const double T6 = T5 * T;
    const double T7 = T6 * T;
    A << T4, T5, T6, T7, 4.0 * T3, 5.0 * T4, 6.0 * T5, 7.0 * T6, 12.0 * T2, 20.0 * T3,
        30.0 * T4, 42.0 * T5, 24.0 * T, 60.0 * T2, 120.0 * T3, 210.0 * T4;
    b << p1 - (c[0] + c[1] * T + c[2] * T2 + c[3] * T3),
        v1 - (c[1] + 2.0 * c[2] * T + 3.0 * c[3] * T2), a1 - (2.0 * c[2] + 6.0 * c[3] * T),
        j1 - (6.0 * c[3]);

    const Eigen::Vector4d tail = A.colPivHouseholderQr().solve(b);
    for (int i = 0; i < 4; ++i) {
        c[static_cast<size_t>(i) + 4U] = tail(i);
    }
    return c;
}

inline void evalSeptic(const std::vector<double>& coeffs, double t, double& p, double& v, double& a,
                       double& j, double& s) {
    p = polyValue(coeffs, t, 0);
    v = polyValue(coeffs, t, 1);
    a = polyValue(coeffs, t, 2);
    j = polyValue(coeffs, t, 3);
    s = polyValue(coeffs, t, 4);
}

inline void fillYawFromVelocity(FlatOutput3& output) {
    const double vx = output.velocity.x();
    const double vy = output.velocity.y();
    const double speed_sq = vx * vx + vy * vy;
    if (speed_sq < 1.0e-8) {
        output.yaw = 0.0;
        output.yaw_rate = 0.0;
        output.yaw_accel = 0.0;
        return;
    }

    const double ax = output.acceleration.x();
    const double ay = output.acceleration.y();
    const double jx = output.jerk.x();
    const double jy = output.jerk.y();
    const double numerator = vx * ay - vy * ax;
    const double numerator_dot = vx * jy - vy * jx;
    const double denominator_dot = 2.0 * (vx * ax + vy * ay);
    output.yaw = std::atan2(vy, vx);
    output.yaw_rate = numerator / speed_sq;
    output.yaw_accel = (numerator_dot * speed_sq - numerator * denominator_dot) / (speed_sq * speed_sq);
}

}  // namespace xgc2_math::trajectory::analytic_detail
