#!/usr/bin/env bash

set -euo pipefail

dpkg -s libxgc2-math-dev >/dev/null
dpkg -s libxgc2-math-core-dev >/dev/null
dpkg -s libxgc2-math-geometry-dev >/dev/null
dpkg -s libxgc2-math-filter-dev >/dev/null
dpkg -s libxgc2-math-observer-dev >/dev/null
dpkg -s libxgc2-math-estimation-dev >/dev/null
dpkg -s libxgc2-math-control-dev >/dev/null

test -f /usr/include/xgc2_math/math.hpp
test -f /usr/include/xgc2_math/types.hpp
test -f /usr/include/xgc2_math/geometry/se2.hpp
test -f /usr/include/xgc2_math/geometry/se3.hpp
test -f /usr/include/xgc2_math/geometry/occupied_sets/sphere_set.h
test -f /usr/include/xgc2_math/filter/exponential_filter.hpp
test -f /usr/include/xgc2_math/observer/differentiator.hpp
test -f /usr/include/xgc2_math/estimation/inertial_pose_eskf.hpp
test -f /usr/include/xgc2_math/estimation/planar_inertial_eskf.hpp
test -f /usr/lib/cmake/xgc2_math/xgc2_mathConfig.cmake

probe_dir="${XGC2_MATH_SMOKE_DIR:-$(mktemp -d -t xgc2-math-smoke-XXXXXX)}"
mkdir -p "${probe_dir}"

cat > "${probe_dir}/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.16)
project(xgc2_math_probe LANGUAGES CXX)

find_package(xgc2_math REQUIRED CONFIG)
xgc2_math_require()

add_executable(link_probe link_probe.cpp)
target_compile_features(link_probe PRIVATE cxx_std_17)
target_link_libraries(link_probe PRIVATE xgc2_math::math)
CMAKE

cat > "${probe_dir}/link_probe.cpp" <<'CPP'
#include <math.hpp>

int main()
{
  xgc2_math::SecondOrderButterworthLowPass filter(5.0, 0.0);
  xgc2_math::ExponentialLowPass smoother(2.0, 0.0);
  xgc2_math::TimeDeltaGuard dt_guard;
  xgc2_math::Differentiator diff;
  xgc2_math::AngleDifferentiator yaw_diff;
  xgc2_math::PositionVelocityLuenbergerObserver observer;
  xgc2_math::AngularPositionVelocityLuenbergerObserver yaw_observer;
  xgc2_math::ArrayDifferentiator<3> array_diff;
  xgc2_math::ScalarRecursiveLeastSquares rls;
  xgc2_math::InertialPoseEskf inertial_pose_eskf;
  xgc2_math::PlanarInertialEskf planar_eskf;
  xgc2_math::SphereSet sphere(Eigen::Vector3d::Zero(), 1.0);
  xgc2_math::Pose2 planar_pose;

  const auto dt0 = dt_guard.update(1.0);
  const auto dt1 = dt_guard.update(1.01);
  const double filtered = filter.filter(1.0, 0.01);
  const double smoothed = smoother.filter(filtered, 0.01);
  const auto derivative = diff.update(smoothed, dt1.dt_s);
  const auto yaw_derivative = yaw_diff.update(xgc2_math::normalizeAngle(3.14), 0.01);
  const auto estimate = observer.update(smoothed, 0.01);
  const auto yaw_estimate = yaw_observer.update(3.14, 0.01);
  const auto array_samples = array_diff.update({{0.0, 1.0, 2.0}}, 0.01);
  rls.reset(2.0);
  const auto rls_sample = rls.update(4.0, 2.0);
  xgc2_math::PoseMeasurement pose_sample;
  pose_sample.received = true;
  pose_sample.valid = true;
  pose_sample.stamp_sec = 1.0;
  inertial_pose_eskf.initializeFromPose(pose_sample, nullptr);
  xgc2_math::PlanarPoseMeasurement planar_sample;
  planar_sample.received = true;
  planar_sample.valid = true;
  planar_sample.stamp_sec = 1.0;
  planar_eskf.initializeFromPose(planar_sample, nullptr);
  planar_pose = xgc2_math::compose(planar_pose, xgc2_math::Pose2{Eigen::Vector2d::UnitX(), 0.1});

  return dt0.accepted && dt1.accepted &&
         derivative.measurement_accepted &&
         yaw_derivative.measurement_accepted &&
         estimate.measurement_accepted &&
         yaw_estimate.measurement_accepted &&
         array_samples[0].measurement_accepted &&
         rls_sample.measurement_accepted &&
         inertial_pose_eskf.initialized() &&
         planar_eskf.initialized() &&
         planar_pose.position.x() > 0.9 &&
         sphere.support(Eigen::Vector3d::UnitX()).support_point.x() > 0.9 ? 0 : 1;
}
CPP

cmake -S "${probe_dir}" -B "${probe_dir}/build"
cmake --build "${probe_dir}/build" -- -j2
"${probe_dir}/build/link_probe"

echo "libxgc2-math-dev installed smoke test passed."
