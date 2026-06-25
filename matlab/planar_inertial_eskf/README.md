# Planar Inertial ESKF Simulation

This directory mirrors the header-only `Pose2InertialEskf` implementation in
`include/estimation/pose2_inertial_eskf.hpp`.

The simulation uses event-driven IMU propagation and VRPN SE2 pose updates. It
is intended as an executable reference for signs, state ordering, covariance
propagation, and innovation handling.

Run in MATLAB or Octave-compatible environments:

```matlab
run_pose2_inertial_eskf_simulation
```
