#pragma once

#include "xgc2_math/trajectory/analytic/detail.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace xgc2_math::trajectory {

struct HelixYzCurveParameters3 {
    uint32_t flags{kFlagNone};
    double duration{25.0};
    Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
    double radius{1.0};
    double omega{1.5};
    double linear_scale{10.0};
    double yaw{0.0};
};

class HelixYzCurveEvaluator3 final : public TrajectoryEvaluator3 {
  public:
    explicit HelixYzCurveEvaluator3(const HelixYzCurveParameters3& params = {}) : params_(params) {
        params_.radius = analytic_detail::safeRadius(params_.radius);
        params_.omega = std::abs(params_.omega);
        params_.linear_scale = std::max(analytic_detail::kMinDuration, std::abs(params_.linear_scale));
        if (!analytic_detail::finiteScalar(params_.omega) || params_.omega <= 0.0) {
            params_.omega = 1.5;
        }
        if (!analytic_detail::finiteScalar(params_.duration) || params_.duration <= 0.0) {
            params_.duration = 25.0;
        }
    }

    bool evaluate(double t, FlatOutput3& output) const override {
        if (!analytic_detail::finiteScalar(t)) {
            output.flags |= kFlagInvalidInput;
            return false;
        }
        t = analytic_detail::clamp(t, 0.0, params_.duration);
        output = FlatOutput3{};
        const double r = params_.radius;
        const double w = params_.omega;
        const double s = params_.linear_scale;
        const double wt = w * t;
        output.position = params_.origin + Eigen::Vector3d(t / s, r * std::cos(wt), r * std::sin(wt));
        output.velocity = Eigen::Vector3d(1.0 / s, -r * w * std::sin(wt), r * w * std::cos(wt));
        output.acceleration = Eigen::Vector3d(0.0, -r * w * w * std::cos(wt), -r * w * w * std::sin(wt));
        output.jerk = Eigen::Vector3d(0.0, r * w * w * w * std::sin(wt), -r * w * w * w * std::cos(wt));
        output.snap = Eigen::Vector3d(0.0, r * w * w * w * w * std::cos(wt), r * w * w * w * w * std::sin(wt));
        analytic_detail::fillYawHold(output, params_.yaw);
        output.flags |= params_.flags;
        return TrajectoryValidator3::finite(output);
    }

    double duration() const override { return params_.duration; }
    TrajectoryModelType type() const override { return TrajectoryModelType::kAnalytic; }
    uint32_t flags() const override { return params_.flags; }
    const HelixYzCurveParameters3& params() const { return params_; }

  private:
    HelixYzCurveParameters3 params_;
};

} // namespace xgc2_math::trajectory
