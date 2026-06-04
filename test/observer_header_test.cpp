#include <cassert>
#include <array>
#include <cmath>
#include <limits>
#include <string>

#include <xgc2_observer/observer.hpp>

namespace {

void testButterworth()
{
    xgc2_observer::SecondOrderButterworthLowPass filter(5.0, 0.0);
    double y = 0.0;
    for (int i = 0; i < 200; ++i) {
        y = filter.filter(1.0, 0.01);
        assert(std::isfinite(y));
    }
    assert(y > 0.95);

    const double held = filter.filter(std::numeric_limits<double>::quiet_NaN(), 0.01);
    assert(std::isfinite(held));
}

void testExponentialLowPass()
{
    xgc2_observer::ExponentialLowPass filter(2.0, 0.0);
    double y = 0.0;
    for (int i = 0; i < 100; ++i) {
        y = filter.filter(1.0, 0.02);
        assert(std::isfinite(y));
    }
    assert(y > 0.9);

    const double held = filter.filter(std::numeric_limits<double>::quiet_NaN(), 0.02);
    assert(held == filter.value());
}

void testStatusHelpers()
{
    assert(xgc2_observer::measurementAccepted(xgc2_observer::SampleStatus::kAccepted));
    assert(xgc2_observer::measurementHeld(xgc2_observer::SampleStatus::kHeldOutlier));
    assert(std::string(xgc2_observer::toString(xgc2_observer::SampleStatus::kHeldInvalidDt)) ==
           "held_invalid_dt");
}

void testTimeDeltaGuard()
{
    xgc2_observer::TimeDeltaGuardOptions options;
    options.min_dt_s = 0.01;
    options.max_dt_s = 0.1;

    xgc2_observer::TimeDeltaGuard guard(options);
    auto sample = guard.update(10.0);
    assert(sample.status == xgc2_observer::SampleStatus::kInitialized);
    assert(sample.accepted);

    sample = guard.update(10.02);
    assert(sample.status == xgc2_observer::SampleStatus::kAccepted);
    assert(std::fabs(sample.dt_s - 0.02) < 1.0e-12);

    sample = guard.update(9.9);
    assert(sample.status == xgc2_observer::SampleStatus::kHeldTimeWentBack);

    sample = guard.update(10.5);
    assert(sample.status == xgc2_observer::SampleStatus::kHeldInvalidDt);
}

void testOptionNormalization()
{
    xgc2_observer::DifferentiatorOptions bad_diff_options;
    bad_diff_options.min_dt_s = -1.0;
    bad_diff_options.max_dt_s = -2.0;
    bad_diff_options.derivative_cutoff_hz = -3.0;
    const auto diff_options = xgc2_observer::normalized(bad_diff_options);
    assert(xgc2_observer::isValid(diff_options));

    xgc2_observer::PositionVelocityObserverOptions bad_observer_options;
    bad_observer_options.position_gain = -1.0;
    bad_observer_options.max_velocity = -1.0;
    const auto observer_options = xgc2_observer::normalized(bad_observer_options);
    assert(xgc2_observer::isValid(observer_options));
}

void testDifferentiator()
{
    xgc2_observer::DifferentiatorOptions options;
    options.min_dt_s = 0.001;
    options.max_dt_s = 0.1;
    options.max_input_step = 1.0;
    options.max_derivative = 20.0;
    options.derivative_cutoff_hz = 10.0;

    xgc2_observer::Differentiator differentiator(options);
    auto sample = differentiator.update(0.0, 0.02);
    assert(sample.status == xgc2_observer::SampleStatus::kInitialized);

    sample = differentiator.update(0.1, 0.02);
    assert(sample.status == xgc2_observer::SampleStatus::kAccepted);
    assert(sample.measurement_accepted);

    const double derivative = sample.derivative;
    sample = differentiator.update(10.0, 0.02);
    assert(sample.status == xgc2_observer::SampleStatus::kHeldOutlier);
    assert(!sample.measurement_accepted);
    assert(std::fabs(sample.derivative - derivative) < 1.0e-12);

    sample = differentiator.update(0.2, 0.0);
    assert(sample.status == xgc2_observer::SampleStatus::kHeldInvalidDt);
}

void testAngleUtilitiesAndDifferentiator()
{
    assert(std::fabs(xgc2_observer::normalizeAngle(3.0 * xgc2_observer::kPi) -
                     xgc2_observer::kPi) < 1.0e-12);

    const double from = xgc2_observer::kPi - 0.01;
    const double to = -xgc2_observer::kPi + 0.01;
    const double delta = xgc2_observer::shortestAngularDistance(from, to);
    assert(std::fabs(delta - 0.02) < 1.0e-12);

    xgc2_observer::DifferentiatorOptions options;
    options.max_input_step = 0.1;
    options.max_derivative = 10.0;

    xgc2_observer::AngleDifferentiator differentiator(options);
    auto sample = differentiator.update(from, 0.02);
    assert(sample.status == xgc2_observer::SampleStatus::kInitialized);

    sample = differentiator.update(to, 0.02);
    assert(sample.status == xgc2_observer::SampleStatus::kAccepted);
    assert(std::fabs(sample.derivative - 1.0) < 1.0e-12);
}

void testPositionVelocityObserver()
{
    xgc2_observer::PositionVelocityObserverOptions options;
    options.position_gain = 0.45;
    options.velocity_gain = 0.12;
    options.max_position_residual = 2.0;
    options.max_velocity = 5.0;

    xgc2_observer::PositionVelocityLuenbergerObserver observer(options);
    auto estimate = observer.update(0.0, 0.02);
    assert(estimate.status == xgc2_observer::SampleStatus::kInitialized);

    for (int i = 1; i <= 200; ++i) {
        estimate = observer.update(0.5 * i * 0.02, 0.02);
        assert(estimate.status == xgc2_observer::SampleStatus::kAccepted);
        assert(std::isfinite(estimate.position));
        assert(std::isfinite(estimate.velocity));
    }

    assert(observer.velocity() > 0.2);
    assert(observer.velocity() < 1.0);

    estimate = observer.update(100.0, 0.02);
    assert(estimate.status == xgc2_observer::SampleStatus::kHeldOutlier);
    assert(!estimate.measurement_accepted);
}

void testAngularPositionVelocityObserver()
{
    xgc2_observer::PositionVelocityObserverOptions options;
    options.position_gain = 0.4;
    options.velocity_gain = 0.1;
    options.max_position_residual = 0.5;

    xgc2_observer::AngularPositionVelocityLuenbergerObserver observer(options);
    auto estimate = observer.update(xgc2_observer::kPi - 0.01, 0.02);
    assert(estimate.status == xgc2_observer::SampleStatus::kInitialized);

    estimate = observer.update(-xgc2_observer::kPi + 0.01, 0.02);
    assert(estimate.status == xgc2_observer::SampleStatus::kAccepted);
    assert(std::fabs(estimate.residual - 0.02) < 1.0e-12);
}

void testArrayWrappers()
{
    xgc2_observer::ArrayDifferentiator<3> differentiator;
    std::array<double, 3> p0{{0.0, 1.0, 2.0}};
    std::array<double, 3> p1{{0.1, 1.2, 2.3}};
    auto samples = differentiator.update(p0, 0.02);
    assert(samples[0].status == xgc2_observer::SampleStatus::kInitialized);
    samples = differentiator.update(p1, 0.1);
    assert(samples[0].status == xgc2_observer::SampleStatus::kAccepted);
    assert(std::fabs(samples[2].derivative - 3.0) < 1.0e-12);

    xgc2_observer::ArrayPositionVelocityLuenbergerObserver<3> observer;
    auto estimates = observer.update(p0, 0.02);
    assert(estimates[1].status == xgc2_observer::SampleStatus::kInitialized);
    estimates = observer.update(p1, 0.02);
    assert(estimates[1].status == xgc2_observer::SampleStatus::kAccepted);
}

}  // namespace

int main()
{
    testButterworth();
    testExponentialLowPass();
    testStatusHelpers();
    testTimeDeltaGuard();
    testOptionNormalization();
    testDifferentiator();
    testAngleUtilitiesAndDifferentiator();
    testPositionVelocityObserver();
    testAngularPositionVelocityObserver();
    testArrayWrappers();
    return 0;
}
