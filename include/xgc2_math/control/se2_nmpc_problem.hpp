#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>

namespace xgc2_math::control {

using Se2StateVector = Eigen::Matrix<double, 4, 1>;
using Se2ControlVector = Eigen::Matrix<double, 2, 1>;

struct Se2State {
    Eigen::Vector2d position{Eigen::Vector2d::Zero()};
    double yaw{0.0};
    double linear_speed{0.0};
};

struct Se2Control {
    double linear_acceleration{0.0};
    double yaw_rate{0.0};
};

struct Se2Reference {
    Se2State state{};
    Se2Control control{};
    uint32_t flags{0U};
};

struct Se2NmpcProblemHorizon {
    std::vector<Se2Reference> references;
    double stage_dt_s{0.0};

    bool validForSteps(int horizon_steps) const {
        return horizon_steps > 0 && stage_dt_s > 0.0 &&
               references.size() >= static_cast<size_t>(horizon_steps + 1);
    }
};

struct Se2NmpcProblemConfig {
    int horizon_steps{10};
    double prediction_horizon_s{1.0};
    double control_period_s{0.1};
    double max_linear_speed{0.0};
    double max_yaw_rate{0.0};
    double max_linear_acceleration{0.0};
};

struct Se2NmpcProblemResult {
    bool success{false};
    int status{0};
    double solve_time_ms{0.0};
    Se2Control first_control{};
    std::vector<Se2State> predicted_states;
    std::vector<Se2Control> predicted_controls;
};

class Se2NmpcProblemBackend {
   public:
    virtual ~Se2NmpcProblemBackend() = default;
    virtual bool initialize() = 0;
    virtual void resetWarmStart() = 0;
    virtual bool solve(const Se2State& initial_state, const Se2NmpcProblemHorizon& horizon) = 0;
    virtual Se2NmpcProblemResult result() const = 0;
    virtual int horizonSteps() const = 0;
};

inline double wrapYaw(double yaw) {
    return std::atan2(std::sin(yaw), std::cos(yaw));
}

inline Se2StateVector packState(const Se2State& state) {
    Se2StateVector value = Se2StateVector::Zero();
    value.segment<2>(0) = state.position;
    value(2) = wrapYaw(state.yaw);
    value(3) = state.linear_speed;
    return value;
}

inline Se2State unpackState(const Se2StateVector& value) {
    Se2State state;
    state.position = value.segment<2>(0);
    state.yaw = wrapYaw(value(2));
    state.linear_speed = value(3);
    return state;
}

inline Se2ControlVector packControl(const Se2Control& control) {
    Se2ControlVector value = Se2ControlVector::Zero();
    value(0) = control.linear_acceleration;
    value(1) = control.yaw_rate;
    return value;
}

inline Se2Control unpackControl(const Se2ControlVector& value) {
    Se2Control control;
    control.linear_acceleration = value(0);
    control.yaw_rate = value(1);
    return control;
}

}  // namespace xgc2_math::control
