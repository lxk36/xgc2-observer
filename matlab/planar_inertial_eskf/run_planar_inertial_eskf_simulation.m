function run_planar_inertial_eskf_simulation()
%RUN_PLANAR_INERTIAL_ESKF_SIMULATION Reference simulation for PlanarInertialEskf.

cfg = default_config();
state = initialize_state(cfg, [0.0; 0.0; 0.0], 0.0);

dt_imu = 0.02;
dt_vrpn = 0.05;
t_final = 5.0;
next_vrpn = 0.0;

truth.p = [0.0; 0.0];
truth.v = [0.5; 0.0];
truth.yaw = 0.2;
gyro_bias_true = 0.01;
accel_bias_true = [0.02; - 0.01];

max_position_error = 0.0;
max_yaw_error = 0.0;

for
    t = 0.0 : dt_imu : t_final truth.p = truth.p + truth.v * dt_imu;

imu.gyro_z = gyro_bias_true;
imu.accel = accel_bias_true;
state = propagate_imu(state, cfg, imu, dt_imu);

if t
    + 1.0e-12 >= next_vrpn marker = compose_pose([truth.p; truth.yaw], cfg.body_to_marker);
measurement = compose_pose(inverse_pose(cfg.measurement_frame_to_world), marker);
[ state, accepted ] = update_vrpn(state, cfg, measurement);
assert(accepted, 'VRPN measurement unexpectedly rejected');
next_vrpn = next_vrpn + dt_vrpn;
end

    max_position_error = max(max_position_error, norm(state.x(1 : 2) - truth.p));
max_yaw_error = max(max_yaw_error, abs(wrap_angle(state.x(5) - truth.yaw)));
end

    assert(max_position_error < 0.20, 'position tracking error too large');
assert(max_yaw_error < 0.20, 'yaw tracking error too large');
assert(all(isfinite(state.x)), 'state contains non-finite values');
assert(all(isfinite(state.P( :))), 'covariance contains non-finite values');

fprintf('Planar inertial ESKF simulation passed. max_position_error=%.4f max_yaw_error=%.4f\n', ... max_position_error,
        max_yaw_error);
end

    function cfg = default_config() cfg.measurement_frame_to_world = [0.0; 0.0; 0.0];
cfg.body_to_marker = [0.2; 0.0; 0.0];
cfg.estimate_extrinsic = false;
cfg.gyro_noise_std = 0.03;
cfg.accel_noise_std = 0.35;
cfg.gyro_bias_random_walk_std = 1.0e-4;
cfg.accel_bias_random_walk_std = 1.0e-3;
cfg.vrpn_position_noise_std = 0.01;
cfg.vrpn_yaw_noise_std = 0.01;
cfg.innovation_position_gate_m = 1.5;
cfg.innovation_yaw_gate_rad = 0.8;
end

    function state = initialize_state(cfg, marker_measurement, stamp) marker_world =
        compose_pose(cfg.measurement_frame_to_world, marker_measurement);
body_world = compose_pose(marker_world, inverse_pose(cfg.body_to_marker));
state.x = zeros(11, 1);
state.x(1 : 2) = body_world(1 : 2);
state.x(5) = body_world(3);
state.x(9 : 10) = cfg.body_to_marker(1 : 2);
state.x(11) = cfg.body_to_marker(3);
state.P = diag([ 0.01, 0.01, 0.1, 0.1, 0.01, 0.01, 0.1, 0.1, 1.0e-12, 1.0e-12, 1.0e-12 ]);
state.last_imu_stamp = stamp;
state.last_pose_stamp = stamp;
end

    function state = propagate_imu(state, cfg, imu, dt) yaw = state.x(5);
gyro_bias = state.x(6);
accel_bias = state.x(7 : 8);
omega_z = imu.gyro_z - gyro_bias;
accel_body = imu.accel - accel_bias;
R = rot2(yaw);
accel_world = R * accel_body;

state.x(1 : 2) = state.x(1 : 2) + state.x(3 : 4) * dt + 0.5 * accel_world * dt * dt;
state.x(3 : 4) = state.x(3 : 4) + accel_world * dt;
state.x(5) = wrap_angle(state.x(5) + omega_z * dt);

J = [ 0, -1; 1, 0 ];
accel_yaw_derivative = R * J * accel_body;
F = eye(11);
F(1 : 2, 3 : 4) = eye(2) * dt;
F(1 : 2, 5) = 0.5 * accel_yaw_derivative * dt * dt;
F(1 : 2, 7 : 8) = -0.5 * R * dt * dt;
F(3 : 4, 5) = accel_yaw_derivative * dt;
F(3 : 4, 7 : 8) = -R * dt;
F(5, 6) = -dt;

Q = zeros(11);
accel_var = cfg.accel_noise_std ^ 2;
gyro_var = cfg.gyro_noise_std ^ 2;
Q(1 : 2, 1 : 2) = eye(2) * 0.25 * accel_var * dt ^ 4;
Q(3 : 4, 3 : 4) = eye(2) * accel_var * dt ^ 2;
Q(5, 5) = gyro_var * dt ^ 2;
Q(6, 6) = cfg.gyro_bias_random_walk_std ^ 2 * dt;
Q(7 : 8, 7 : 8) = eye(2) * cfg.accel_bias_random_walk_std ^ 2 * dt;

state.P = F * state.P * F' + Q;
state.P = 0.5 * (state.P + state.P');
end

function [state, accepted] = update_vrpn(state, cfg, measurement)
predicted_marker = predicted_marker_pose(state);
innovation = se2_error(predicted_marker, compose_pose(cfg.measurement_frame_to_world, measurement));

if norm(innovation(1:2)) > cfg.innovation_position_gate_m || abs(innovation(3)) > cfg.innovation_yaw_gate_rad
    accepted = false;
    return;
end

H = numerical_jacobian(state, cfg, measurement, predicted_marker);
R = diag([cfg.vrpn_position_noise_std^2, cfg.vrpn_position_noise_std^2, cfg.vrpn_yaw_noise_std^2]);
S = H * state.P * H' + R;
K = state.P * H' / S;
delta = K * innovation;
state = inject_error(state, cfg, delta);
I = eye(11);
state.P = (I - K * H) * state.P * (I - K * H)' + K * R * K';
state.P = 0.5 * (state.P + state.P');
accepted = true;
end

function H = numerical_jacobian(state, cfg, measurement, nominal_prediction)
eps = 1.0e-6;
marker_world = compose_pose(cfg.measurement_frame_to_world, measurement);
nominal_residual = se2_error(nominal_prediction, marker_world);
H = zeros(3, 11);
for i = 1:11
    delta = zeros(11, 1);
    delta(i) = eps;
    perturbed = inject_error(state, cfg, delta);
    residual = se2_error(predicted_marker_pose(perturbed), marker_world);
    H(:, i) = -(residual - nominal_residual) / eps;
end
if ~cfg.estimate_extrinsic
    H(:, 9:11) = 0.0;
end
end

function state = inject_error(state, cfg, delta)
state.x(1:2) = state.x(1:2) + delta(1:2);
state.x(3:4) = state.x(3:4) + delta(3:4);
state.x(5) = wrap_angle(state.x(5) + delta(5));
state.x(6) = state.x(6) + delta(6);
state.x(7:8) = state.x(7:8) + delta(7:8);
if cfg.estimate_extrinsic
    state.x(9:10) = state.x(9:10) + delta(9:10);
    state.x(11) = wrap_angle(state.x(11) + delta(11));
end
end

function marker = predicted_marker_pose(state)
body = [state.x(1:2); state.x(5)];
body_to_marker = [state.x(9:10); state.x(11)];
marker = compose_pose(body, body_to_marker);
end

function c = compose_pose(a, b)
c = zeros(3, 1);
c(1:2) = a(1:2) + rot2(a(3)) * b(1:2);
c(3) = wrap_angle(a(3) + b(3));
end

function inv = inverse_pose(a)
inv = zeros(3, 1);
inv(3) = wrap_angle(-a(3));
inv(1:2) = -rot2(inv(3)) * a(1:2);
end

function e = se2_error(reference, measured)
delta = compose_pose(inverse_pose(reference), measured);
e = [delta(1:2); wrap_angle(delta(3))];
end

function R = rot2(yaw)
c = cos(yaw);
s = sin(yaw);
R = [c, -s; s, c];
end

function value = wrap_angle(value)
value = atan2(sin(value), cos(value));
end
