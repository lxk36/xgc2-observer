# Inertial Pose ESKF MATLAB Simulation

`run_inertial_pose_eskf_simulation.m` mirrors the C++ observer tests for the
ROS-independent `xgc2_observer::InertialPoseEskf` algorithm. It uses the same
state variables, SE3 composition convention, innovation gates, quaternion
normalization rule, and non-ideal input cases.

Run in MATLAB:

```matlab
results = run_inertial_pose_eskf_simulation();
disp(results)
```

The script is intentionally self-contained so it can be used as a numerical
reference when changing the C++ header-only implementation.
