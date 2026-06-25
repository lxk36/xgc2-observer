#pragma once

#include "xgc2_math/trajectory/analytic/detail.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace xgc2_math::trajectory {

struct CircleCurveParameters2 {
    uint32_t flags{kFlagNone};
    double duration{60.0};
    Eigen::Vector2d center{Eigen::Vector2d::Zero()};
    double radius{3.0};
    double line_speed{1.0};
};

class CircleCurveEvaluator2 final : public TrajectoryEvaluator2 {
   public:
    explicit CircleCurveEvaluator2(CircleCurveParameters2 params = {}) : params_(params) {
        params_.radius = analytic_detail::safeRadius(params_.radius);
        params_.line_speed = std::max(0.0, params_.line_speed);
        if (!analytic_detail::finiteScalar(params_.duration) || params_.duration <= 0.0) {
            params_.duration = 60.0;
        }
    }

    bool evaluate(double t, PlanarReference2& output) const override {
        if (!analytic_detail::finiteScalar(t)) {
            output.flags |= kFlagInvalidInput;
            return false;
        }
        t = analytic_detail::clamp(t, 0.0, params_.duration);
        output = PlanarReference2{};
        evaluateCircle(t, output);
        output.flags |= params_.flags;
        return TrajectoryValidator2::finite(output);
    }

    void evaluateCircle(double t, PlanarReference2& output) const {
        const double r = params_.radius;
        const double w = analytic_detail::angularRate(params_.radius, params_.line_speed);
        const double wt = w * t;
        output.position.x() = params_.center.x() + r * std::cos(wt);
        output.position.y() = params_.center.y() + r * std::sin(wt);
        output.velocity.x() = -r * w * std::sin(wt);
        output.velocity.y() = r * w * std::cos(wt);
        output.acceleration.x() = -r * w * w * std::cos(wt);
        output.acceleration.y() = -r * w * w * std::sin(wt);
        output.jerk.x() = r * w * w * w * std::sin(wt);
        output.jerk.y() = -r * w * w * w * std::cos(wt);
        completePlanarReference2(output);
    }

    double duration() const override {
        return params_.duration;
    }
    TrajectoryModelType type() const override {
        return TrajectoryModelType::kAnalytic;
    }
    uint32_t flags() const override {
        return params_.flags;
    }
    const CircleCurveParameters2& params() const {
        return params_;
    }

   private:
    CircleCurveParameters2 params_;
};

struct CircleCurveParameters3 {
    uint32_t flags{kFlagNone};
    double duration{60.0};
    Eigen::Vector2d center{Eigen::Vector2d::Zero()};
    double radius{3.0};
    double line_speed{3.0};
    double height{3.0};
    double z_amplitude{0.0};
    double z_frequency{0.5};
};

class CircleCurveEvaluator3 final : public TrajectoryEvaluator3 {
   public:
    explicit CircleCurveEvaluator3(CircleCurveParameters3 params = {}) : params_(params) {
        params_.radius = analytic_detail::safeRadius(params_.radius);
        params_.line_speed = std::max(0.0, params_.line_speed);
        if (!analytic_detail::finiteScalar(params_.duration) || params_.duration <= 0.0) {
            params_.duration = 60.0;
        }
        if (!analytic_detail::finiteScalar(params_.z_frequency) || params_.z_frequency <= 0.0) {
            params_.z_frequency = 0.5 * analytic_detail::angularRate(params_.radius, params_.line_speed);
        }
    }

    bool evaluate(double t, FlatOutput3& output) const override {
        if (!analytic_detail::finiteScalar(t)) {
            output.flags |= kFlagInvalidInput;
            return false;
        }
        t = analytic_detail::clamp(t, 0.0, params_.duration);
        output = FlatOutput3{};
        evaluateCircle(t, output);
        output.flags |= params_.flags;
        return TrajectoryValidator3::finite(output);
    }

    void evaluateCircle(double t, FlatOutput3& output) const {
        const double r = params_.radius;
        const double w = analytic_detail::angularRate(params_.radius, params_.line_speed);
        const double z_w = params_.z_frequency;
        const double wt = w * t;
        const double zt = z_w * t;
        const bool height_axis = std::abs(params_.z_amplitude) > 0.0;
        output.position.x() = params_.center.x() + r * std::cos(wt);
        output.position.y() = params_.center.y() + r * std::sin(wt);
        output.position.z() = params_.height + (height_axis ? params_.z_amplitude * std::sin(zt) : 0.0);
        output.velocity.x() = -r * w * std::sin(wt);
        output.velocity.y() = r * w * std::cos(wt);
        output.velocity.z() = height_axis ? params_.z_amplitude * z_w * std::cos(zt) : 0.0;
        output.acceleration.x() = -r * w * w * std::cos(wt);
        output.acceleration.y() = -r * w * w * std::sin(wt);
        output.acceleration.z() = height_axis ? -params_.z_amplitude * z_w * z_w * std::sin(zt) : 0.0;
        output.jerk.x() = r * w * w * w * std::sin(wt);
        output.jerk.y() = -r * w * w * w * std::cos(wt);
        output.jerk.z() = height_axis ? -params_.z_amplitude * z_w * z_w * z_w * std::cos(zt) : 0.0;
        output.snap.x() = r * w * w * w * w * std::cos(wt);
        output.snap.y() = r * w * w * w * w * std::sin(wt);
        output.snap.z() = height_axis ? params_.z_amplitude * z_w * z_w * z_w * z_w * std::sin(zt) : 0.0;
        analytic_detail::fillYawFromVelocity(output);
    }

    double duration() const override {
        return params_.duration;
    }
    TrajectoryModelType type() const override {
        return TrajectoryModelType::kAnalytic;
    }
    uint32_t flags() const override {
        return params_.flags;
    }
    const CircleCurveParameters3& params() const {
        return params_;
    }

   private:
    CircleCurveParameters3 params_;
};

}  // namespace xgc2_math::trajectory
