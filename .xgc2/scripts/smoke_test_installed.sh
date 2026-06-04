#!/usr/bin/env bash

set -euo pipefail

dpkg -s libxgc2-observer-dev >/dev/null

test -f /usr/include/xgc2_observer/observer.hpp
test -f /usr/lib/cmake/xgc2_observer/xgc2_observerConfig.cmake

probe_dir="${XGC2_OBSERVER_SMOKE_DIR:-$(mktemp -d -t xgc2-observer-smoke-XXXXXX)}"
mkdir -p "${probe_dir}"

cat > "${probe_dir}/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.16)
project(xgc2_observer_probe LANGUAGES CXX)

find_package(xgc2_observer REQUIRED CONFIG)
xgc2_observer_require()

add_executable(link_probe link_probe.cpp)
target_compile_features(link_probe PRIVATE cxx_std_17)
target_link_libraries(link_probe PRIVATE xgc2_observer::observer)
CMAKE

cat > "${probe_dir}/link_probe.cpp" <<'CPP'
#include <xgc2_observer/observer.hpp>

int main()
{
  xgc2_observer::SecondOrderButterworthLowPass filter(5.0, 0.0);
  xgc2_observer::ExponentialLowPass smoother(2.0, 0.0);
  xgc2_observer::TimeDeltaGuard dt_guard;
  xgc2_observer::Differentiator diff;
  xgc2_observer::AngleDifferentiator yaw_diff;
  xgc2_observer::PositionVelocityLuenbergerObserver observer;
  xgc2_observer::AngularPositionVelocityLuenbergerObserver yaw_observer;
  xgc2_observer::ArrayDifferentiator<3> array_diff;
  xgc2_observer::ScalarRecursiveLeastSquares rls;

  const auto dt0 = dt_guard.update(1.0);
  const auto dt1 = dt_guard.update(1.01);
  const double filtered = filter.filter(1.0, 0.01);
  const double smoothed = smoother.filter(filtered, 0.01);
  const auto derivative = diff.update(smoothed, dt1.dt_s);
  const auto yaw_derivative = yaw_diff.update(xgc2_observer::normalizeAngle(3.14), 0.01);
  const auto estimate = observer.update(smoothed, 0.01);
  const auto yaw_estimate = yaw_observer.update(3.14, 0.01);
  const auto array_samples = array_diff.update({{0.0, 1.0, 2.0}}, 0.01);
  rls.reset(2.0);
  const auto rls_sample = rls.update(4.0, 2.0);

  return dt0.accepted && dt1.accepted &&
         derivative.measurement_accepted &&
         yaw_derivative.measurement_accepted &&
         estimate.measurement_accepted &&
         yaw_estimate.measurement_accepted &&
         array_samples[0].measurement_accepted &&
         rls_sample.measurement_accepted ? 0 : 1;
}
CPP

cmake -S "${probe_dir}" -B "${probe_dir}/build"
cmake --build "${probe_dir}/build" -- -j2
"${probe_dir}/build/link_probe"

echo "libxgc2-observer-dev installed smoke test passed."
