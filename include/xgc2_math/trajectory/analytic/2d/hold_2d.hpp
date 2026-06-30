#pragma once

#include "xgc2_math/trajectory/analytic/detail.hpp"

#include <Eigen/Dense>
#include <cstdint>

namespace xgc2_math::trajectory {

struct HoldCurveParameters2 {
    uint32_t flags{kFlagNone};
    double duration{60.0};
    Eigen::Vector2d position{Eigen::Vector2d::Zero()};
    double yaw{0.0};
};

class HoldCurveEvaluator2 final : public TrajectoryEvaluator2 {
  public:
    explicit HoldCurveEvaluator2(const HoldCurveParameters2& params = {}) : params_(params) {
        if (!analytic_detail::finiteScalar(params_.duration) || params_.duration <= 0.0) {
            params_.duration = 60.0;
        }
    }

    bool evaluate(double t, PlanarReference2& output) const override {
        if (!analytic_detail::finiteScalar(t)) {
            output.flags |= kFlagInvalidInput;
            return false;
        }
        output = PlanarReference2{};
        output.position = params_.position;
        output.yaw = params_.yaw;
        completePlanarReference2(output);
        output.flags |= params_.flags;
        return TrajectoryValidator2::finite(output);
    }

    double duration() const override { return params_.duration; }
    TrajectoryModelType type() const override { return TrajectoryModelType::kAnalytic; }
    uint32_t flags() const override { return params_.flags; }
    const HoldCurveParameters2& params() const { return params_; }

  private:
    HoldCurveParameters2 params_;
};

} // namespace xgc2_math::trajectory
