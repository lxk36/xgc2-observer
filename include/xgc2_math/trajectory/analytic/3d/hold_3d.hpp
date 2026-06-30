#pragma once

#include "xgc2_math/trajectory/analytic/detail.hpp"

#include <Eigen/Dense>
#include <cstdint>

namespace xgc2_math::trajectory {

struct HoldCurveParameters3 {
    uint32_t flags{kFlagNone};
    double duration{60.0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    double yaw{0.0};
};

class HoldCurveEvaluator3 final : public TrajectoryEvaluator3 {
  public:
    explicit HoldCurveEvaluator3(const HoldCurveParameters3& params = {}) : params_(params) {
        if (!analytic_detail::finiteScalar(params_.duration) || params_.duration <= 0.0) {
            params_.duration = 60.0;
        }
    }

    bool evaluate(double t, FlatOutput3& output) const override {
        if (!analytic_detail::finiteScalar(t)) {
            output.flags |= kFlagInvalidInput;
            return false;
        }
        output = FlatOutput3{};
        output.position = params_.position;
        output.yaw = params_.yaw;
        output.flags |= params_.flags;
        return TrajectoryValidator3::finite(output);
    }

    double duration() const override { return params_.duration; }
    TrajectoryModelType type() const override { return TrajectoryModelType::kAnalytic; }
    uint32_t flags() const override { return params_.flags; }
    const HoldCurveParameters3& params() const { return params_; }

  private:
    HoldCurveParameters3 params_;
};

} // namespace xgc2_math::trajectory
