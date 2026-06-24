#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>

#include <math.hpp>

namespace {

void expect(bool condition) {
    if (!condition) {
        std::abort();
    }
}

void testButterworth() {
    xgc2_math::SecondOrderButterworthLowPass filter(5.0, 0.0);
    double y = 0.0;
    for (int i = 0; i < 200; ++i) {
        y = filter.filter(1.0, 0.01);
        expect(std::isfinite(y));
    }
    expect(y > 0.95);

    const double held = filter.filter(std::numeric_limits<double>::quiet_NaN(), 0.01);
    expect(std::isfinite(held));
}

void testExponentialLowPass() {
    xgc2_math::ExponentialLowPass filter(2.0, 0.0);
    double y = 0.0;
    for (int i = 0; i < 100; ++i) {
        y = filter.filter(1.0, 0.02);
        expect(std::isfinite(y));
    }
    expect(y > 0.9);

    const double held = filter.filter(std::numeric_limits<double>::quiet_NaN(), 0.02);
    expect(held == filter.value());
}

void testSlewRateLimiter() {
    xgc2_math::SlewRateLimiter limiter(0.5, 0.0);

    expect(std::fabs(limiter.filter(10.0, 0.2) - 0.1) < 1.0e-12);
    expect(std::fabs(limiter.filter(-10.0, 0.2) - 0.0) < 1.0e-12);
    const double held = limiter.filter(std::numeric_limits<double>::quiet_NaN(), 0.2);
    expect(held == limiter.value());

    limiter.setMaxRatePerSecond(0.0);
    expect(std::fabs(limiter.filter(2.0, 0.2) - 2.0) < 1.0e-12);
}

void testStatusHelpers() {
    expect(xgc2_math::measurementAccepted(xgc2_math::SampleStatus::kAccepted));
    expect(xgc2_math::measurementHeld(xgc2_math::SampleStatus::kHeldOutlier));
    expect(std::string(xgc2_math::toString(xgc2_math::SampleStatus::kHeldInvalidDt)) == "held_invalid_dt");
}

void testTimeDeltaGuard() {
    xgc2_math::TimeDeltaGuardOptions options;
    options.min_dt_s = 0.01;
    options.max_dt_s = 0.1;

    xgc2_math::TimeDeltaGuard guard(options);
    auto sample = guard.update(10.0);
    expect(sample.status == xgc2_math::SampleStatus::kInitialized);
    expect(sample.accepted);

    sample = guard.update(10.02);
    expect(sample.status == xgc2_math::SampleStatus::kAccepted);
    expect(std::fabs(sample.dt_s - 0.02) < 1.0e-12);

    sample = guard.update(9.9);
    expect(sample.status == xgc2_math::SampleStatus::kHeldTimeWentBack);

    sample = guard.update(10.5);
    expect(sample.status == xgc2_math::SampleStatus::kHeldInvalidDt);
}

void testOptionNormalization() {
    xgc2_math::DifferentiatorOptions bad_diff_options;
    bad_diff_options.min_dt_s = -1.0;
    bad_diff_options.max_dt_s = -2.0;
    bad_diff_options.max_input_step = std::numeric_limits<double>::quiet_NaN();
    bad_diff_options.derivative_cutoff_hz = -3.0;
    const auto diff_options = xgc2_math::normalized(bad_diff_options);
    expect(xgc2_math::isValid(diff_options));

    xgc2_math::PositionVelocityObserverOptions bad_observer_options;
    bad_observer_options.position_gain = -1.0;
    bad_observer_options.max_velocity = std::numeric_limits<double>::quiet_NaN();
    const auto observer_options = xgc2_math::normalized(bad_observer_options);
    expect(xgc2_math::isValid(observer_options));
}

void testDifferentiator() {
    xgc2_math::DifferentiatorOptions options;
    options.min_dt_s = 0.001;
    options.max_dt_s = 0.1;
    options.max_input_step = 1.0;
    options.max_derivative = 20.0;
    options.derivative_cutoff_hz = 10.0;

    xgc2_math::Differentiator differentiator(options);
    auto sample = differentiator.update(0.0, 0.02);
    expect(sample.status == xgc2_math::SampleStatus::kInitialized);

    sample = differentiator.update(0.1, 0.02);
    expect(sample.status == xgc2_math::SampleStatus::kAccepted);
    expect(sample.measurement_accepted);

    const double derivative = sample.derivative;
    sample = differentiator.update(10.0, 0.02);
    expect(sample.status == xgc2_math::SampleStatus::kHeldOutlier);
    expect(!sample.measurement_accepted);
    expect(std::fabs(sample.derivative - derivative) < 1.0e-12);

    sample = differentiator.update(0.2, 0.0);
    expect(sample.status == xgc2_math::SampleStatus::kHeldInvalidDt);
}

void testAngleUtilitiesAndDifferentiator() {
    expect(std::fabs(xgc2_math::normalizeAngle(3.0 * xgc2_math::kPi) - xgc2_math::kPi) < 1.0e-12);

    const double from = xgc2_math::kPi - 0.01;
    const double to = -xgc2_math::kPi + 0.01;
    const double delta = xgc2_math::shortestAngularDistance(from, to);
    expect(std::fabs(delta - 0.02) < 1.0e-12);

    xgc2_math::DifferentiatorOptions options;
    options.max_input_step = 0.1;
    options.max_derivative = 10.0;

    xgc2_math::AngleDifferentiator differentiator(options);
    auto sample = differentiator.update(from, 0.02);
    expect(sample.status == xgc2_math::SampleStatus::kInitialized);

    sample = differentiator.update(to, 0.02);
    expect(sample.status == xgc2_math::SampleStatus::kAccepted);
    expect(std::fabs(sample.derivative - 1.0) < 1.0e-12);
}

void testPositionVelocityObserver() {
    xgc2_math::PositionVelocityObserverOptions options;
    options.position_gain = 0.45;
    options.velocity_gain = 0.12;
    options.max_position_residual = 2.0;
    options.max_velocity = 5.0;

    xgc2_math::PositionVelocityLuenbergerObserver observer(options);
    auto estimate = observer.update(0.0, 0.02);
    expect(estimate.status == xgc2_math::SampleStatus::kInitialized);

    for (int i = 1; i <= 200; ++i) {
        estimate = observer.update(0.5 * i * 0.02, 0.02);
        expect(estimate.status == xgc2_math::SampleStatus::kAccepted);
        expect(std::isfinite(estimate.position));
        expect(std::isfinite(estimate.velocity));
    }

    expect(observer.velocity() > 0.2);
    expect(observer.velocity() < 1.0);

    estimate = observer.update(100.0, 0.02);
    expect(estimate.status == xgc2_math::SampleStatus::kHeldOutlier);
    expect(!estimate.measurement_accepted);
}

void testAngularPositionVelocityObserver() {
    xgc2_math::PositionVelocityObserverOptions options;
    options.position_gain = 0.4;
    options.velocity_gain = 0.1;
    options.max_position_residual = 0.5;

    xgc2_math::AngularPositionVelocityLuenbergerObserver observer(options);
    auto estimate = observer.update(xgc2_math::kPi - 0.01, 0.02);
    expect(estimate.status == xgc2_math::SampleStatus::kInitialized);

    estimate = observer.update(-xgc2_math::kPi + 0.01, 0.02);
    expect(estimate.status == xgc2_math::SampleStatus::kAccepted);
    expect(std::fabs(estimate.residual - 0.02) < 1.0e-12);
}

void testArrayWrappers() {
    xgc2_math::ArrayDifferentiator<3> differentiator;
    std::array<double, 3> p0{{0.0, 1.0, 2.0}};
    std::array<double, 3> p1{{0.1, 1.2, 2.3}};
    auto samples = differentiator.update(p0, 0.02);
    expect(samples[0].status == xgc2_math::SampleStatus::kInitialized);
    samples = differentiator.update(p1, 0.1);
    expect(samples[0].status == xgc2_math::SampleStatus::kAccepted);
    expect(std::fabs(samples[2].derivative - 3.0) < 1.0e-12);

    xgc2_math::ArrayPositionVelocityLuenbergerObserver<3> observer;
    auto estimates = observer.update(p0, 0.02);
    expect(estimates[1].status == xgc2_math::SampleStatus::kInitialized);
    estimates = observer.update(p1, 0.02);
    expect(estimates[1].status == xgc2_math::SampleStatus::kAccepted);
}

void testScalarRecursiveLeastSquares() {
    xgc2_math::ScalarRecursiveLeastSquaresOptions bad_options;
    bad_options.forgetting_factor = 2.0;
    bad_options.initial_covariance = std::numeric_limits<double>::infinity();
    bad_options.min_abs_regressor = -1.0;
    const auto options = xgc2_math::normalized(bad_options);
    expect(xgc2_math::isValid(options));

    xgc2_math::ScalarRecursiveLeastSquares estimator(options);
    auto sample = estimator.update(4.0, 2.0);
    expect(sample.status == xgc2_math::SampleStatus::kHeldInvalidInput);

    estimator.reset(1.0, 10.0);
    sample = estimator.update(6.0, 2.0);
    expect(sample.status == xgc2_math::SampleStatus::kAccepted);
    expect(sample.measurement_accepted);
    expect(sample.parameter > 2.0);
    expect(sample.parameter < 3.0);
    expect(sample.covariance > 0.0);

    const double held_parameter = sample.parameter;
    sample = estimator.update(std::numeric_limits<double>::quiet_NaN(), 2.0);
    expect(sample.status == xgc2_math::SampleStatus::kHeldInvalidInput);
    expect(std::fabs(sample.parameter - held_parameter) < 1.0e-12);

    sample = estimator.update(6.0, 0.0);
    expect(sample.status == xgc2_math::SampleStatus::kHeldInvalidInput);

    sample = estimator.update(6.0, std::numeric_limits<double>::quiet_NaN());
    expect(sample.status == xgc2_math::SampleStatus::kHeldInvalidInput);
    expect(!sample.measurement_accepted);
}

struct InertialPoseTestSamples {
    static xgc2_math::InertialSample inertial(double stamp_sec, const Eigen::Vector3d& gyro = Eigen::Vector3d::Zero(),
                                              const Eigen::Vector3d& accel = Eigen::Vector3d(0.0, 0.0, 9.8066)) {
        xgc2_math::InertialSample sample;
        sample.received = true;
        sample.valid = true;
        sample.stamp_sec = stamp_sec;
        sample.angular_velocity = gyro;
        sample.linear_acceleration = accel;
        return sample;
    }

    static xgc2_math::PoseMeasurement pose(double stamp_sec, const Eigen::Vector3d& position,
                                           const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity()) {
        xgc2_math::PoseMeasurement sample;
        sample.received = true;
        sample.valid = true;
        sample.stamp_sec = stamp_sec;
        sample.pose.position = position;
        sample.pose.orientation = xgc2_math::normalizedQuaternion(orientation);
        return sample;
    }
};

void testSe3Utilities() {
    const Eigen::Quaterniond q = xgc2_math::normalizedQuaternion(Eigen::Quaterniond(-2.0, 0.1, -0.2, 0.3));
    expect(std::fabs(q.norm() - 1.0) < 1.0e-12);
    expect(q.w() >= 0.0);

    const Eigen::Quaterniond invalid =
        xgc2_math::normalizedQuaternion(Eigen::Quaterniond(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0));
    expect(std::fabs(invalid.w() - 1.0) < 1.0e-12);
    expect(invalid.vec().norm() < 1.0e-12);

    xgc2_math::Pose3 transform;
    transform.position = Eigen::Vector3d(1.0, 2.0, 3.0);
    transform.orientation = xgc2_math::rpyToQuaternion(Eigen::Vector3d(0.1, -0.2, 0.3));
    const xgc2_math::Pose3 identity = xgc2_math::compose(transform, xgc2_math::inverse(transform));
    expect(identity.position.norm() < 1.0e-12);
    expect(std::fabs(identity.orientation.w() - 1.0) < 1.0e-12);

    xgc2_math::Pose3 delta;
    delta.position = Eigen::Vector3d(0.2, -0.1, 0.05);
    delta.orientation = xgc2_math::expMap(Eigen::Vector3d(0.03, -0.02, 0.01));
    const xgc2_math::Pose3 measured = xgc2_math::compose(transform, delta);

    const Eigen::Matrix<double, 6, 1> error = xgc2_math::se3Error(transform, measured);
    expect((error.head<3>() - delta.position).norm() < 1.0e-12);
    expect((error.tail<3>() - xgc2_math::logMap(delta.orientation)).norm() < 1.0e-12);
}

void testSe2Utilities() {
    xgc2_math::Pose2 transform;
    transform.position = Eigen::Vector2d(1.0, -0.5);
    transform.yaw = 0.4;

    const xgc2_math::Pose2 identity = xgc2_math::compose(transform, xgc2_math::inverse(transform));
    expect(identity.position.norm() < 1.0e-12);
    expect(std::fabs(identity.yaw) < 1.0e-12);

    xgc2_math::Pose2 delta;
    delta.position = Eigen::Vector2d(0.2, -0.1);
    delta.yaw = 0.03;
    const xgc2_math::Pose2 measured = xgc2_math::compose(transform, delta);
    const Eigen::Vector3d error = xgc2_math::se2Error(transform, measured);
    expect((error.head<2>() - delta.position).norm() < 1.0e-12);
    expect(std::fabs(error.z() - delta.yaw) < 1.0e-12);
}

void testPlanarInertialEskf() {
    xgc2_math::PlanarInertialEskfConfig config;
    config.measurement_frame_to_world.position = Eigen::Vector2d(1.0, 2.0);
    config.measurement_frame_to_world.yaw = 0.0;
    config.body_to_marker.position = Eigen::Vector2d(0.2, 0.0);
    config.vrpn_position_noise_std = 0.01;
    config.vrpn_yaw_noise_std = 0.01;
    config.accel_noise_std = 0.3;
    config.gyro_noise_std = 0.03;
    config.gyro_bias_random_walk_std = 1.0e-4;
    config.accel_bias_random_walk_std = 1.0e-3;
    config.innovation_position_gate_m = 0.8;
    config.innovation_yaw_gate_rad = 0.5;

    xgc2_math::PlanarInertialEskf estimator;
    estimator.setConfig(config);

    xgc2_math::PlanarPoseMeasurement first_pose;
    first_pose.received = true;
    first_pose.valid = true;
    first_pose.stamp_sec = 1.0;
    first_pose.pose.position = Eigen::Vector2d(2.0, 0.0);
    first_pose.pose.yaw = 0.1;
    xgc2_math::PlanarInertialSample first_imu;
    first_imu.received = true;
    first_imu.valid = true;
    first_imu.stamp_sec = 1.0;
    first_imu.angular_velocity_z = 0.0;
    first_imu.linear_acceleration = Eigen::Vector2d::Zero();
    estimator.initializeFromPose(first_pose, &first_imu);

    expect(estimator.initialized());
    const xgc2_math::Pose2 expected_marker_world =
        xgc2_math::compose(config.measurement_frame_to_world, first_pose.pose);
    const xgc2_math::Pose2 expected_body_world =
        xgc2_math::compose(expected_marker_world, xgc2_math::inverse(config.body_to_marker));
    expect((estimator.state().position - expected_body_world.position).norm() < 1.0e-12);
    expect(std::fabs(estimator.state().yaw - 0.1) < 1.0e-12);
    expect(estimator.hasCorrectedBodyPose());

    xgc2_math::PlanarInertialSample imu = first_imu;
    imu.stamp_sec = 1.02;
    imu.angular_velocity_z = 0.02;
    imu.linear_acceleration = Eigen::Vector2d(0.1, 0.0);
    const auto propagated = estimator.propagateInertial(imu);
    expect(propagated.accepted);
    expect(estimator.state().position.x() > expected_body_world.position.x());
    expect(estimator.state().yaw > 0.1);

    xgc2_math::PlanarPoseMeasurement second_pose = first_pose;
    second_pose.stamp_sec = 1.1;
    second_pose.pose.position = Eigen::Vector2d(2.2, 0.0);
    second_pose.pose.yaw = 0.12;
    const auto accepted = estimator.updatePose(second_pose);
    expect(accepted.accepted);
    expect(!accepted.innovation_rejected);
    expect(estimator.state().position.x() > 2.8);
    expect(estimator.state().velocity.x() > 0.0);
    expect(estimator.state().yaw_rate > 0.0);
    expect(estimator.covariance().trace() > 0.0);

    const Eigen::Vector2d held_position = estimator.state().position;
    xgc2_math::PlanarPoseMeasurement jump_pose = second_pose;
    jump_pose.stamp_sec = 1.2;
    jump_pose.pose.position = Eigen::Vector2d(10.0, 0.0);
    const auto rejected = estimator.updatePose(jump_pose);
    expect(!rejected.accepted);
    expect(rejected.innovation_rejected);
    expect((estimator.state().position - held_position).norm() < 1.0e-12);
}

void testInertialPoseEskfInitialization() {
    xgc2_math::InertialPoseEskfConfig config;
    config.measurement_frame_to_world.position = Eigen::Vector3d(1.0, 2.0, 3.0);
    config.body_to_marker.position = Eigen::Vector3d(0.1, 0.0, 0.0);

    xgc2_math::InertialPoseEskf eskf;
    eskf.setConfig(config);

    const auto imu = InertialPoseTestSamples::inertial(1.0);
    const auto pose = InertialPoseTestSamples::pose(1.0, Eigen::Vector3d(2.0, 0.0, 1.0));
    eskf.initializeFromPose(pose, &imu);

    expect(eskf.initialized());
    expect(std::fabs(eskf.state().position.x() - 2.9) < 1.0e-12);
    expect(std::fabs(eskf.state().position.y() - 2.0) < 1.0e-12);
    expect(std::fabs(eskf.state().position.z() - 4.0) < 1.0e-12);
    expect(eskf.hasCorrectedBodyPose());
    expect(std::fabs(eskf.correctedBodyPose().position.x() - 2.9) < 1.0e-12);
}

void testInertialPoseEskfStationaryPropagation() {
    xgc2_math::InertialPoseEskf eskf;
    const auto imu0 = InertialPoseTestSamples::inertial(1.0, Eigen::Vector3d(0.1, 0.0, 0.0));
    eskf.initializeFromPose(InertialPoseTestSamples::pose(1.0, Eigen::Vector3d::Zero()), &imu0);

    const auto imu1 = InertialPoseTestSamples::inertial(1.01, Eigen::Vector3d(0.1, 0.0, 0.0));
    eskf.propagateInertial(imu1);

    expect(std::fabs(eskf.state().angular_velocity.x() - 0.1) < 1.0e-12);
    expect(eskf.state().velocity.norm() < 1.0e-3);
    expect(std::fabs(eskf.state().orientation.norm() - 1.0) < 1.0e-12);
    expect(eskf.state().orientation.w() >= 0.0);
}

void testInertialPoseEskfPoseUpdateAndReject() {
    xgc2_math::InertialPoseEskfConfig config;
    config.innovation_position_gate_m = 0.5;

    xgc2_math::InertialPoseEskf eskf;
    eskf.setConfig(config);
    const auto imu0 = InertialPoseTestSamples::inertial(1.0);
    eskf.initializeFromPose(InertialPoseTestSamples::pose(1.0, Eigen::Vector3d::Zero()), &imu0);

    auto result = eskf.updatePose(InertialPoseTestSamples::pose(1.02, Eigen::Vector3d(0.4, 0.0, 0.0)));
    expect(result.accepted);
    expect(eskf.state().position.x() > 0.3);
    expect(eskf.state().position.x() < 0.4);

    const double held_position = eskf.state().position.x();
    result = eskf.updatePose(InertialPoseTestSamples::pose(1.04, Eigen::Vector3d(2.0, 0.0, 0.0)));
    expect(!result.accepted);
    expect(result.innovation_rejected);
    expect(std::fabs(eskf.state().position.x() - held_position) < 1.0e-12);
}

void testInertialPoseEskfInvalidAndLargeDtHoldState() {
    xgc2_math::InertialPoseEskf eskf;
    const auto imu0 = InertialPoseTestSamples::inertial(1.0);
    eskf.initializeFromPose(InertialPoseTestSamples::pose(1.0, Eigen::Vector3d::Zero()), &imu0);

    xgc2_math::InertialSample invalid_imu = InertialPoseTestSamples::inertial(1.01);
    invalid_imu.angular_velocity.x() = std::numeric_limits<double>::quiet_NaN();
    eskf.propagateInertial(invalid_imu);
    expect(std::fabs(eskf.state().last_inertial_stamp_sec - 1.0) < 1.0e-12);

    eskf.propagateInertial(InertialPoseTestSamples::inertial(2.0));
    expect(std::fabs(eskf.state().last_inertial_stamp_sec - 2.0) < 1.0e-12);
    expect(eskf.state().position.norm() < 1.0e-12);

    xgc2_math::PoseMeasurement invalid_pose = InertialPoseTestSamples::pose(2.01, Eigen::Vector3d::Zero());
    invalid_pose.pose.position.z() = std::numeric_limits<double>::quiet_NaN();
    const auto result = eskf.updatePose(invalid_pose);
    expect(!result.accepted);
    expect(!result.innovation_rejected);
    expect(eskf.state().position.norm() < 1.0e-12);
}

void testInertialPoseEskfGyroBiasAndCorrectedPose() {
    xgc2_math::InertialPoseEskfConfig config;
    config.position_update_gain = 0.0;
    config.velocity_update_gain = 0.0;
    config.orientation_update_gain = 0.5;
    config.gyro_bias_update_gain = 0.01;

    xgc2_math::InertialPoseEskf eskf;
    eskf.setConfig(config);
    const auto imu0 = InertialPoseTestSamples::inertial(1.0);
    eskf.initializeFromPose(InertialPoseTestSamples::pose(1.0, Eigen::Vector3d::Zero()), &imu0);

    const auto yaw_pose = InertialPoseTestSamples::pose(1.1, Eigen::Vector3d(0.2, 0.0, 0.0),
                                                        xgc2_math::rpyToQuaternion(Eigen::Vector3d(0.0, 0.0, 0.1)));
    const auto result = eskf.updatePose(yaw_pose);
    expect(result.accepted);
    expect(std::fabs(eskf.correctedBodyPose().position.x() - 0.2) < 1.0e-12);
    expect(std::fabs(eskf.state().position.x()) < 1.0e-12);
    expect(eskf.state().gyro_bias.z() < 0.0);
    expect(eskf.state().orientation.w() >= 0.0);
}

} // namespace

int main() {
    testButterworth();
    testExponentialLowPass();
    testSlewRateLimiter();
    testStatusHelpers();
    testTimeDeltaGuard();
    testOptionNormalization();
    testDifferentiator();
    testAngleUtilitiesAndDifferentiator();
    testPositionVelocityObserver();
    testAngularPositionVelocityObserver();
    testArrayWrappers();
    testScalarRecursiveLeastSquares();
    testSe3Utilities();
    testSe2Utilities();
    testPlanarInertialEskf();
    testInertialPoseEskfInitialization();
    testInertialPoseEskfStationaryPropagation();
    testInertialPoseEskfPoseUpdateAndReject();
    testInertialPoseEskfInvalidAndLargeDtHoldState();
    testInertialPoseEskfGyroBiasAndCorrectedPose();
    return 0;
}
