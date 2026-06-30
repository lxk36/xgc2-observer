#pragma once

#include "xgc2_math/trajectory/analytic/2d/circle_2d.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cstdint>

namespace xgc2_math::trajectory {

struct CircleEntryCurveParameters2 {
    uint32_t flags{kFlagNone};
    double duration{60.0};
    Eigen::Vector2d origin{Eigen::Vector2d::Zero()};
    double origin_yaw{0.0};
    double entry_duration{3.0};
    CircleCurveParameters2 circle{};
};

class CircleEntryCurveEvaluator2 final : public TrajectoryEvaluator2 {
  public:
    explicit CircleEntryCurveEvaluator2(const CircleEntryCurveParameters2& params = {})
        : params_(params), circle_(params_.circle) {
        params_.entry_duration = std::max(0.0, params_.entry_duration);
        if (!analytic_detail::finiteScalar(params_.duration) || params_.duration <= 0.0) {
            params_.duration = circle_.duration() + params_.entry_duration;
        }
    }

    bool evaluate(double t, PlanarReference2& output) const override {
        if (!analytic_detail::finiteScalar(t)) {
            output.flags |= kFlagInvalidInput;
            return false;
        }
        t = analytic_detail::clamp(t, 0.0, params_.duration);
        const double entry = std::max(analytic_detail::kMinDuration, params_.entry_duration);
        output = PlanarReference2{};
        if (t >= entry) {
            const bool ok = circle_.evaluate(t - entry, output);
            output.flags |= params_.flags;
            return ok && TrajectoryValidator2::finite(output);
        }

        PlanarReference2 end;
        circle_.evaluateCircle(0.0, end);
        const auto cx = analytic_detail::septicBoundary(params_.origin.x(), 0.0, 0.0, 0.0, end.position.x(),
                                                        end.velocity.x(), end.acceleration.x(), end.jerk.x(), entry);
        const auto cy = analytic_detail::septicBoundary(params_.origin.y(), 0.0, 0.0, 0.0, end.position.y(),
                                                        end.velocity.y(), end.acceleration.y(), end.jerk.y(), entry);
        output.position << analytic_detail::polyValue(cx, t, 0), analytic_detail::polyValue(cy, t, 0);
        output.velocity << analytic_detail::polyValue(cx, t, 1), analytic_detail::polyValue(cy, t, 1);
        output.acceleration << analytic_detail::polyValue(cx, t, 2), analytic_detail::polyValue(cy, t, 2);
        output.jerk << analytic_detail::polyValue(cx, t, 3), analytic_detail::polyValue(cy, t, 3);
        completePlanarReference2(output);
        const double progress = t / entry;
        const double blend = analytic_detail::smoothstep01(progress);
        const double blend_dot = analytic_detail::smoothstep01Derivative(progress) / entry;
        const double blend_ddot = analytic_detail::smoothstep01SecondDerivative(progress) / (entry * entry);
        analytic_detail::velocityYawWithBlend(output.velocity, output.acceleration, output.jerk, params_.origin_yaw,
                                              blend, blend_dot, blend_ddot, output.yaw, output.yaw_rate,
                                              output.yaw_acceleration);
        output.curvature = output.speed > 1.0e-6 ? output.yaw_rate / output.speed : 0.0;
        output.flags |= params_.flags;
        return TrajectoryValidator2::finite(output);
    }

    double duration() const override { return params_.duration; }
    TrajectoryModelType type() const override { return TrajectoryModelType::kAnalytic; }
    uint32_t flags() const override { return params_.flags | circle_.flags(); }
    const CircleEntryCurveParameters2& params() const { return params_; }

  private:
    CircleEntryCurveParameters2 params_;
    CircleCurveEvaluator2 circle_;
};

} // namespace xgc2_math::trajectory
