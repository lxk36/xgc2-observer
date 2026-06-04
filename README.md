# XGC2 Observer

`libxgc2-observer-dev` provides small C++17 numerical observer utilities for
robotics runtime code.  The package is ROS-independent so ROS1, ROS2, Gazebo,
PX4/MAVROS integration code, controllers, MATLAB bindings, and non-ROS CMake
projects can use the same installed headers.

The scope is deliberately narrow:

- status helpers for logging and diagnostics;
- timestamp-to-`dt` guarding for runtime data streams;
- low-pass filtering for weak timing jitter and noisy scalar signals;
- numerical differentiation that rejects invalid samples, bad `dt`, and large
  jumps before updating state;
- angle wrapping and shortest angular distance utilities for yaw-like signals;
- position/velocity Luenberger-style observers for motion-capture and target
  tracking signals;
- scalar recursive least-squares estimation for one-parameter online models;
- lightweight `std::array` wrappers for fixed-size multi-axis use.

## Install

```bash
sudo apt update
sudo apt install libxgc2-observer-dev
```

The package installs:

```text
/usr/include/xgc2_observer/
/usr/lib/cmake/xgc2_observer/
```

## CMake Usage

```cmake
find_package(xgc2_observer REQUIRED CONFIG)

target_link_libraries(your_target
  PRIVATE
    xgc2_observer::observer
)
```

Include the umbrella header:

```cpp
#include <xgc2_observer/observer.hpp>
```

or individual headers:

```cpp
#include <xgc2_observer/butterworth_filter.hpp>
#include <xgc2_observer/differentiator.hpp>
#include <xgc2_observer/luenberger_observer.hpp>
```

## Algorithms

### `SampleStatus`

Observer updates return a `SampleStatus` so callers can publish diagnostics
without duplicating status mapping code.

```cpp
const auto sample = diff.update(position, dt_s);
ROS_DEBUG_STREAM("observer status=" << xgc2_observer::toString(sample.status));

if (xgc2_observer::measurementHeld(sample.status)) {
  // Keep the previous controller input or report degraded sensor quality.
}
```

### `TimeDeltaGuard`

`TimeDeltaGuard` converts monotonically increasing timestamps into checked
sampling intervals.  It rejects invalid timestamps, time jumps backwards, too
small intervals, and too large intervals before caller state is updated.

```cpp
xgc2_observer::TimeDeltaGuardOptions options;
options.min_dt_s = 0.001;
options.max_dt_s = 0.1;

xgc2_observer::TimeDeltaGuard dt_guard(options);
const auto dt = dt_guard.update(stamp.toSec());
if (dt.accepted && dt.status == xgc2_observer::SampleStatus::kAccepted) {
  diff.update(position, dt.dt_s);
}
```

### `SecondOrderButterworthLowPass`

Second-order Butterworth low-pass filter with coefficients recomputed from the
current `dt`.  This makes it usable with small period jitter while preserving a
simple scalar API.

```cpp
xgc2_observer::SecondOrderButterworthLowPass filter(5.0, 0.0);
const double filtered = filter.filter(raw_value, dt_s);
```

Invalid input holds the last output.  Invalid `dt` or non-positive cutoff resets
the filter state to the current input because filtering semantics are not
defined in that case.

### `ExponentialLowPass`

First-order exponential low-pass filter for simple command smoothing or noisy
signals where a second-order filter is unnecessary.

```cpp
xgc2_observer::ExponentialLowPass filter(2.0, 0.0);
const double filtered = filter.filter(raw_value, dt_s);
```

### `Differentiator`

Derivative estimator that updates only when the input and sampling interval pass
sanity checks.

```cpp
xgc2_observer::DifferentiatorOptions options;
options.max_input_step = 0.5;
options.max_derivative = 5.0;
options.derivative_cutoff_hz = 8.0;

xgc2_observer::Differentiator diff(options);
const auto sample = diff.update(position, dt_s);
```

The returned sample reports whether the measurement was accepted or held because
of invalid input, invalid `dt`, or an outlier.  State is updated only for
accepted samples.

Options are normalized on construction and in `setOptions()`.  Invalid `dt`
limits, negative cutoff values, and negative bounds are replaced with conservative
defaults instead of letting invalid configuration propagate into runtime math.

### Angle Utilities

Yaw and heading signals must be treated on the unit circle.  Use the angle
helpers before computing residuals manually.

```cpp
const double yaw = xgc2_observer::normalizeAngle(raw_yaw);
const double error = xgc2_observer::shortestAngularDistance(current_yaw, target_yaw);
```

`AngleDifferentiator` uses the same options as `Differentiator`, but computes
the derivative from shortest angular distance, so crossing `pi` does not create
a false derivative spike.

```cpp
xgc2_observer::AngleDifferentiator yaw_rate(options);
const auto sample = yaw_rate.update(yaw_rad, dt_s);
```

### `PositionVelocityLuenbergerObserver`

One-dimensional position/velocity observer for target tracking and motion-capture
signals.  It predicts with optional acceleration input, then corrects position
and velocity from measured position residual.

```cpp
xgc2_observer::PositionVelocityObserverOptions options;
options.position_gain = 0.35;
options.velocity_gain = 0.08;
options.max_position_residual = 1.0;

xgc2_observer::PositionVelocityLuenbergerObserver observer(options);
const auto estimate = observer.update(measured_position, dt_s);
```

`AngularPositionVelocityLuenbergerObserver` has the same interface, but applies
angle wrapping to the predicted angle and shortest-distance residual.  Use it
for yaw or other continuous revolute coordinates.

### `ScalarRecursiveLeastSquares`

One-dimensional recursive least-squares estimator for online models of the
form:

```text
y = phi * theta
```

where `theta` is the scalar parameter being estimated.  The estimator owns the
parameter, covariance, forgetting factor, and basic numeric guards.  Domain
packages should still own their physical interpretation, unit conversions, and
operational gating.

```cpp
xgc2_observer::ScalarRecursiveLeastSquaresOptions options;
options.forgetting_factor = 0.998;
options.initial_covariance = 100.0;

xgc2_observer::ScalarRecursiveLeastSquares rls(options);
rls.reset(initial_parameter);

const auto sample = rls.update(measurement, regressor);
if (sample.measurement_accepted) {
  const double theta = sample.parameter;
}
```

For hover-thrust estimation, a caller can map `theta` to thrust-to-acceleration:

```text
acc_z = normalized_thrust * thrust_to_acceleration
hover_thrust = gravity / thrust_to_acceleration
```

### Fixed-Size Array Wrappers

Scalar observers remain the core API.  For fixed-size vectors, use the
`std::array` wrappers to avoid duplicating per-axis loops in every caller.

```cpp
xgc2_observer::ArrayDifferentiator<3> velocity_from_position;
std::array<double, 3> position{{x, y, z}};
const auto samples = velocity_from_position.update(position, dt_s);

xgc2_observer::ArrayPositionVelocityLuenbergerObserver<3> mocap_observer;
const auto estimates = mocap_observer.update(position, dt_s);
```

## Design Boundary

This package owns numerical signal conditioning and observer primitives.  It
does not own ROS topics, frame transforms, MAVROS routing, Gazebo model state,
or controller policy.  Callers are expected to set limits according to their
sensor and vehicle envelope.

## Build Locally

```bash
cmake -S . -B build
cmake --build build -- -j"$(nproc)"
(cd build && ctest --output-on-failure)
```

Package scripts:

```bash
./.xgc2/scripts/check_package_compliance.sh
./.xgc2/scripts/build_deb.sh
sudo apt-get install -y ./.ci/debs/libxgc2-observer-dev_*.deb
./.xgc2/scripts/smoke_test_installed.sh
```
