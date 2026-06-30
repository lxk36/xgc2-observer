#pragma once

#include "xgc2_math/trajectory/analytic/detail.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace xgc2_math::trajectory {

struct TorusKnotCurveParameters3 {
    uint32_t flags{kFlagNone};
    double duration{35.0};
    Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
    double omega{0.9};
    double scale{0.3};
    double yaw{0.0};
};

class TorusKnotCurveEvaluator3 final : public TrajectoryEvaluator3 {
  public:
    explicit TorusKnotCurveEvaluator3(const TorusKnotCurveParameters3& params = {}) : params_(params) {
        params_.omega = std::abs(params_.omega);
        params_.scale = std::abs(params_.scale);
        if (!analytic_detail::finiteScalar(params_.omega) || params_.omega <= 0.0) {
            params_.omega = 0.9;
        }
        if (!analytic_detail::finiteScalar(params_.scale) || params_.scale <= 0.0) {
            params_.scale = 0.3;
        }
        if (!analytic_detail::finiteScalar(params_.duration) || params_.duration <= 0.0) {
            params_.duration = 35.0;
        }
    }

    bool evaluate(double t, FlatOutput3& output) const override {
        if (!analytic_detail::finiteScalar(t)) {
            output.flags |= kFlagInvalidInput;
            return false;
        }
        t = analytic_detail::clamp(t, 0.0, params_.duration);
        output = FlatOutput3{};
        const double w = params_.omega;
        const double sc = params_.scale;
        const double wt = w * t;
        output.position =
            params_.origin + sc * Eigen::Vector3d(std::sin(wt) + 2.0 * std::sin(2.0 * wt),
                                                  std::cos(wt) - 2.0 * std::cos(2.0 * wt), 4.0 + std::sin(3.0 * wt));
        output.velocity =
            sc * Eigen::Vector3d(w * std::cos(wt) + 4.0 * w * std::cos(2.0 * wt),
                                 -w * std::sin(wt) + 4.0 * w * std::sin(2.0 * wt), 3.0 * w * std::cos(3.0 * wt));
        output.acceleration = sc * Eigen::Vector3d(-w * w * std::sin(wt) - 8.0 * w * w * std::sin(2.0 * wt),
                                                   -w * w * std::cos(wt) + 8.0 * w * w * std::cos(2.0 * wt),
                                                   -9.0 * w * w * std::sin(3.0 * wt));
        output.jerk = sc * Eigen::Vector3d(-w * w * w * std::cos(wt) - 16.0 * w * w * w * std::cos(2.0 * wt),
                                           w * w * w * std::sin(wt) - 16.0 * w * w * w * std::sin(2.0 * wt),
                                           -27.0 * w * w * w * std::cos(3.0 * wt));
        output.snap = sc * Eigen::Vector3d(w * w * w * w * std::sin(wt) + 32.0 * w * w * w * w * std::sin(2.0 * wt),
                                           w * w * w * w * std::cos(wt) - 32.0 * w * w * w * w * std::cos(2.0 * wt),
                                           81.0 * w * w * w * w * std::sin(3.0 * wt));
        analytic_detail::fillYawHold(output, params_.yaw);
        output.flags |= params_.flags;
        return TrajectoryValidator3::finite(output);
    }

    double duration() const override { return params_.duration; }
    TrajectoryModelType type() const override { return TrajectoryModelType::kAnalytic; }
    uint32_t flags() const override { return params_.flags; }
    const TorusKnotCurveParameters3& params() const { return params_; }

  private:
    TorusKnotCurveParameters3 params_;
};

} // namespace xgc2_math::trajectory
