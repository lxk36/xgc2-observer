#pragma once

#include "xgc2_math/trajectory/analytic/detail.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace xgc2_math::trajectory {

struct FigureEightCurveParameters3 {
    uint32_t flags{kFlagNone};
    double duration{60.0};
    Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
    double radius{3.0};
    double line_speed{3.0};
    double height{3.0};
    double yaw{0.0};
};

class FigureEightCurveEvaluator3 final : public TrajectoryEvaluator3 {
  public:
    explicit FigureEightCurveEvaluator3(const FigureEightCurveParameters3& params = {}) : params_(params) {
        params_.radius = analytic_detail::safeRadius(params_.radius);
        params_.line_speed = std::max(0.0, params_.line_speed);
        if (!analytic_detail::finiteScalar(params_.duration) || params_.duration <= 0.0) {
            params_.duration = 60.0;
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
        const double w = analytic_detail::angularRate(params_.radius, params_.line_speed);
        const double wt = w * t;
        output.position.x() = params_.origin.x() + r * std::sin(wt);
        output.position.y() = params_.origin.y() + 0.5 * r * std::sin(2.0 * wt);
        output.position.z() = params_.height;
        output.velocity.x() = r * w * std::cos(wt);
        output.velocity.y() = r * w * std::cos(2.0 * wt);
        output.acceleration.x() = -r * w * w * std::sin(wt);
        output.acceleration.y() = -2.0 * r * w * w * std::sin(2.0 * wt);
        output.jerk.x() = -r * w * w * w * std::cos(wt);
        output.jerk.y() = -4.0 * r * w * w * w * std::cos(2.0 * wt);
        output.snap.x() = r * w * w * w * w * std::sin(wt);
        output.snap.y() = 8.0 * r * w * w * w * w * std::sin(2.0 * wt);
        analytic_detail::fillYawHold(output, params_.yaw);
        output.flags |= params_.flags;
        return TrajectoryValidator3::finite(output);
    }

    double duration() const override { return params_.duration; }
    TrajectoryModelType type() const override { return TrajectoryModelType::kAnalytic; }
    uint32_t flags() const override { return params_.flags; }
    const FigureEightCurveParameters3& params() const { return params_; }

  private:
    FigureEightCurveParameters3 params_;
};

} // namespace xgc2_math::trajectory
