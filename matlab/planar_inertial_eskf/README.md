# Planar Inertial ESKF Simulation

This directory mirrors the header-only `PlanarInertialEskf` implementation in
`include/estimation/planar_inertial_eskf.hpp`.

The simulation uses event-driven IMU propagation and VRPN SE2 pose updates. It
is intended as an executable reference for signs, state ordering, covariance
propagation, and innovation handling.

Run in MATLAB or Octave-compatible environments:

```matlab
run_planar_inertial_eskf_simulation
```
