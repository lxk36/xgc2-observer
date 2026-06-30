#pragma once

#include "xgc2_math/trajectory/analytic/detail.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace xgc2_math::trajectory {

struct LemniscateCurveParameters3 {
    uint32_t flags{kFlagNone};
    double duration{20.0};
    Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
    double radius{1.0};
    double omega{0.9};
    double height{1.0};
    double yaw{0.0};
};

class LemniscateCurveEvaluator3 final : public TrajectoryEvaluator3 {
  public:
    explicit LemniscateCurveEvaluator3(const LemniscateCurveParameters3& params = {}) : params_(params) {
        params_.radius = analytic_detail::safeRadius(params_.radius);
        params_.omega = std::abs(params_.omega);
        if (!analytic_detail::finiteScalar(params_.omega) || params_.omega <= 0.0) {
            params_.omega = 0.9;
        }
        if (!analytic_detail::finiteScalar(params_.duration) || params_.duration <= 0.0) {
            params_.duration = 20.0;
        }
    }

    bool evaluate(double t, FlatOutput3& output) const override {
        if (!analytic_detail::finiteScalar(t)) {
            output.flags |= kFlagInvalidInput;
            return false;
        }
        t = analytic_detail::clamp(t, 0.0, params_.duration);
        output = FlatOutput3{};
        const double a = params_.radius;
        const double w = params_.omega;
        const double wt = w * t;
        const double sin_wt = std::sin(wt);
        const double cos_wt = std::cos(wt);
        const double sin_2wt = std::sin(2.0 * wt);
        const double cos_2wt = std::cos(2.0 * wt);
        output.position = params_.origin + Eigen::Vector3d(a * sin_wt, a * sin_wt * cos_wt, params_.height);
        output.velocity = Eigen::Vector3d(a * w * cos_wt, a * w * cos_2wt, 0.0);
        output.acceleration = Eigen::Vector3d(-a * w * w * sin_wt, -2.0 * a * w * w * sin_2wt, 0.0);
        output.jerk = Eigen::Vector3d(-a * w * w * w * cos_wt, -4.0 * a * w * w * w * cos_2wt, 0.0);
        output.snap = Eigen::Vector3d(a * w * w * w * w * sin_wt, 8.0 * a * w * w * w * w * sin_2wt, 0.0);
        analytic_detail::fillYawHold(output, params_.yaw);
        output.flags |= params_.flags;
        return TrajectoryValidator3::finite(output);
    }

    double duration() const override { return params_.duration; }
    TrajectoryModelType type() const override { return TrajectoryModelType::kAnalytic; }
    uint32_t flags() const override { return params_.flags; }
    const LemniscateCurveParameters3& params() const { return params_; }

  private:
    LemniscateCurveParameters3 params_;
};

} // namespace xgc2_math::trajectory
