#pragma once

#include "xgc2_math/trajectory/analytic/circle.hpp"

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
    explicit CircleEntryCurveEvaluator2(CircleEntryCurveParameters2 params = {})
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
        const auto cx = analytic_detail::septicBoundary(params_.origin.x(), 0.0, 0.0, 0.0,
                                                        end.position.x(), end.velocity.x(),
                                                        end.acceleration.x(), end.jerk.x(), entry);
        const auto cy = analytic_detail::septicBoundary(params_.origin.y(), 0.0, 0.0, 0.0,
                                                        end.position.y(), end.velocity.y(),
                                                        end.acceleration.y(), end.jerk.y(), entry);
        output.position << analytic_detail::polyValue(cx, t, 0), analytic_detail::polyValue(cy, t, 0);
        output.velocity << analytic_detail::polyValue(cx, t, 1), analytic_detail::polyValue(cy, t, 1);
        output.acceleration << analytic_detail::polyValue(cx, t, 2), analytic_detail::polyValue(cy, t, 2);
        output.jerk << analytic_detail::polyValue(cx, t, 3), analytic_detail::polyValue(cy, t, 3);
        output.yaw = params_.origin_yaw;
        completePlanarReference2(output);
        output.flags |= params_.flags;
        return TrajectoryValidator2::finite(output);
    }

    double duration() const override {
        return params_.duration;
    }
    TrajectoryModelType type() const override {
        return TrajectoryModelType::kAnalytic;
    }
    uint32_t flags() const override {
        return params_.flags | circle_.flags();
    }
    const CircleEntryCurveParameters2& params() const {
        return params_;
    }

   private:
    CircleEntryCurveParameters2 params_;
    CircleCurveEvaluator2 circle_;
};

struct CircleEntryCurveParameters3 {
    uint32_t flags{kFlagNone};
    double duration{60.0};
    Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
    double origin_yaw{0.0};
    double entry_duration{5.0};
    CircleCurveParameters3 circle{};
};

class CircleEntryCurveEvaluator3 final : public TrajectoryEvaluator3 {
   public:
    explicit CircleEntryCurveEvaluator3(CircleEntryCurveParameters3 params = {})
        : params_(params), circle_(params_.circle) {
        params_.entry_duration = std::max(0.0, params_.entry_duration);
        if (!analytic_detail::finiteScalar(params_.duration) || params_.duration <= 0.0) {
            params_.duration = circle_.duration() + params_.entry_duration;
        }
    }

    bool evaluate(double t, FlatOutput3& output) const override {
        if (!analytic_detail::finiteScalar(t)) {
            output.flags |= kFlagInvalidInput;
            return false;
        }
        t = analytic_detail::clamp(t, 0.0, params_.duration);
        const double entry = std::max(analytic_detail::kMinDuration, params_.entry_duration);
        output = FlatOutput3{};
        if (t >= entry) {
            const bool ok = circle_.evaluate(t - entry, output);
            output.flags |= params_.flags;
            return ok && TrajectoryValidator3::finite(output);
        }

        FlatOutput3 end;
        circle_.evaluateCircle(0.0, end);
        const Eigen::Vector3d start(params_.origin.x(), params_.origin.y(), params_.circle.height);
        const auto cx = analytic_detail::septicBoundary(start.x(), 0.0, 0.0, 0.0, end.position.x(),
                                                        end.velocity.x(), end.acceleration.x(), end.jerk.x(),
                                                        entry);
        const auto cy = analytic_detail::septicBoundary(start.y(), 0.0, 0.0, 0.0, end.position.y(),
                                                        end.velocity.y(), end.acceleration.y(), end.jerk.y(),
                                                        entry);
        const auto cz = analytic_detail::septicBoundary(start.z(), 0.0, 0.0, 0.0, end.position.z(),
                                                        end.velocity.z(), end.acceleration.z(), end.jerk.z(),
                                                        entry);
        analytic_detail::evalSeptic(cx, t, output.position.x(), output.velocity.x(), output.acceleration.x(),
                                    output.jerk.x(), output.snap.x());
        analytic_detail::evalSeptic(cy, t, output.position.y(), output.velocity.y(), output.acceleration.y(),
                                    output.jerk.y(), output.snap.y());
        analytic_detail::evalSeptic(cz, t, output.position.z(), output.velocity.z(), output.acceleration.z(),
                                    output.jerk.z(), output.snap.z());
        analytic_detail::fillYawFromVelocity(output);
        if (output.velocity.head<2>().squaredNorm() < 1.0e-8) {
            output.yaw = params_.origin_yaw;
        }
        output.flags |= params_.flags;
        return TrajectoryValidator3::finite(output);
    }

    double duration() const override {
        return params_.duration;
    }
    TrajectoryModelType type() const override {
        return TrajectoryModelType::kAnalytic;
    }
    uint32_t flags() const override {
        return params_.flags | circle_.flags();
    }
    const CircleEntryCurveParameters3& params() const {
        return params_;
    }

   private:
    CircleEntryCurveParameters3 params_;
    CircleCurveEvaluator3 circle_;
};

}  // namespace xgc2_math::trajectory
