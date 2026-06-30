#pragma once

#include "xgc2_math/trajectory/analytic/detail.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cstdint>

namespace xgc2_math::trajectory {

struct LineCurveParameters3 {
    uint32_t flags{kFlagNone};
    double duration{8.0};
    Eigen::Vector3d start{Eigen::Vector3d::Zero()};
    Eigen::Vector3d start_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d target{Eigen::Vector3d(1.0, 1.0, 1.0)};
    Eigen::Vector3d target_velocity{Eigen::Vector3d::Zero()};
    double yaw{0.0};
};

class LineCurveEvaluator3 final : public TrajectoryEvaluator3 {
  public:
    explicit LineCurveEvaluator3(const LineCurveParameters3& params = {}) : params_(params) {
        if (!analytic_detail::finiteScalar(params_.duration) || params_.duration <= 0.0) {
            params_.duration = 8.0;
        }
        const double T = std::max(analytic_detail::kMinDuration, params_.duration);
        const Eigen::Vector3d a_delta = params_.target - params_.start - params_.start_velocity * T;
        const Eigen::Vector3d d_delta = params_.start_velocity - params_.target_velocity;
        a0_ = params_.start;
        a1_ = params_.start_velocity;
        a3_ = (10.0 * a_delta + 4.0 * d_delta * T) / (T * T * T);
        a4_ = -(15.0 * a_delta + 7.0 * d_delta * T) / (T * T * T * T);
        a5_ = (3.0 * (2.0 * a_delta + d_delta * T)) / (T * T * T * T * T);
    }

    bool evaluate(double t, FlatOutput3& output) const override {
        if (!analytic_detail::finiteScalar(t)) {
            output.flags |= kFlagInvalidInput;
            return false;
        }
        t = analytic_detail::clamp(t, 0.0, params_.duration);
        output = FlatOutput3{};
        const double t2 = t * t;
        const double t3 = t2 * t;
        const double t4 = t3 * t;
        const double t5 = t4 * t;
        output.position = a0_ + a1_ * t + a3_ * t3 + a4_ * t4 + a5_ * t5;
        output.velocity = a1_ + 3.0 * a3_ * t2 + 4.0 * a4_ * t3 + 5.0 * a5_ * t4;
        output.acceleration = 6.0 * a3_ * t + 12.0 * a4_ * t2 + 20.0 * a5_ * t3;
        output.jerk = 6.0 * a3_ + 24.0 * a4_ * t + 60.0 * a5_ * t2;
        output.snap = 24.0 * a4_ + 120.0 * a5_ * t;
        analytic_detail::fillYawHold(output, params_.yaw);
        output.flags |= params_.flags;
        return TrajectoryValidator3::finite(output);
    }

    double duration() const override { return params_.duration; }
    TrajectoryModelType type() const override { return TrajectoryModelType::kAnalytic; }
    uint32_t flags() const override { return params_.flags; }
    const LineCurveParameters3& params() const { return params_; }

  private:
    LineCurveParameters3 params_;
    Eigen::Vector3d a0_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d a1_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d a3_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d a4_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d a5_{Eigen::Vector3d::Zero()};
};

} // namespace xgc2_math::trajectory
