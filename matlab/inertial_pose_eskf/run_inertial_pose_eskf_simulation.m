function results = run_pose3_inertial_eskf_simulation()
%RUN_INERTIAL_POSE_ESKF_SIMULATION Reference scenarios for Pose3InertialEskf.
% Quaternion convention is [w x y z]. Pose convention is T_AB: frame B in A.

cfg = default_config();
results = struct();

results.se3 = scenario_se3();
results.initialization = scenario_initialization(cfg);
results.stationary = scenario_stationary(cfg);
results.pose_update_and_reject = scenario_pose_update_and_reject(cfg);
results.invalid_and_large_dt = scenario_invalid_and_large_dt(cfg);
results.bias_and_corrected_pose = scenario_bias_and_corrected_pose(cfg);
end

    function result = scenario_se3() q = quat_normalize([-2.0 0.1 - 0.2 0.3]);
assert(abs(norm(q) - 1.0) < 1.0e-12);
assert(q(1) >= 0.0);

T.position = [1.0; 2.0; 3.0];
T.orientation = rpy_to_quat([0.1; - 0.2; 0.3]);
I = pose_compose(T, pose_inverse(T));
assert(norm(I.position) < 1.0e-12);
assert(abs(I.orientation(1) - 1.0) < 1.0e-12);
result = true;
end

    function result = scenario_initialization(cfg) cfg.measurement_frame_to_world.position = [1.0; 2.0; 3.0];
cfg.body_to_marker.position = [0.1; 0.0; 0.0];
eskf = reset_eskf(cfg);

imu = inertial_sample(1.0, [0; 0; 0], [0; 0; 9.8066]);
pose = pose_sample(1.0, [2.0; 0.0; 1.0], [1 0 0 0]);
eskf = initialize_from_pose(eskf, pose, imu);

assert(eskf.state.initialized);
assert(abs(eskf.state.position(1) - 2.9) < 1.0e-12);
assert(abs(eskf.state.position(2) - 2.0) < 1.0e-12);
assert(abs(eskf.state.position(3) - 4.0) < 1.0e-12);
result = eskf.state.position;
end

    function result = scenario_stationary(cfg) eskf = reset_eskf(cfg);
imu0 = inertial_sample(1.0, [0.1; 0; 0], [0; 0; 9.8066]);
eskf = initialize_from_pose(eskf, pose_sample(1.0, [0; 0; 0], [1 0 0 0]), imu0);
eskf = propagate_inertial(eskf, inertial_sample(1.01, [0.1; 0; 0], [0; 0; 9.8066]));

assert(abs(eskf.state.angular_velocity(1) - 0.1) < 1.0e-12);
assert(norm(eskf.state.velocity) < 1.0e-3);
assert(abs(norm(eskf.state.orientation) - 1.0) < 1.0e-12);
result = eskf.state.velocity;
end

    function result = scenario_pose_update_and_reject(cfg) cfg.innovation_position_gate_m = 0.5;
eskf = reset_eskf(cfg);
eskf = initialize_from_pose(eskf, pose_sample(1.0, [0; 0; 0], [1 0 0 0]),
                            ... inertial_sample(1.0, [0; 0; 0], [0; 0; 9.8066]));

[ eskf, accepted, rejected ] = update_pose(eskf, pose_sample(1.02, [0.4; 0; 0], [1 0 0 0]));
assert(accepted && ~rejected);
assert(eskf.state.position(1) > 0.3 && eskf.state.position(1) < 0.4);
held_position = eskf.state.position(1);

[ eskf, accepted, rejected ] = update_pose(eskf, pose_sample(1.04, [2.0; 0; 0], [1 0 0 0]));
assert(~accepted&& rejected);
assert(abs(eskf.state.position(1) - held_position) < 1.0e-12);
result = eskf.state.position;
end

    function result = scenario_invalid_and_large_dt(cfg) eskf = reset_eskf(cfg);
eskf = initialize_from_pose(eskf, pose_sample(1.0, [0; 0; 0], [1 0 0 0]),
                            ... inertial_sample(1.0, [0; 0; 0], [0; 0; 9.8066]));

bad_imu = inertial_sample(1.01, [NaN; 0; 0], [0; 0; 9.8066]);
eskf = propagate_inertial(eskf, bad_imu);
assert(abs(eskf.state.last_inertial_stamp_sec - 1.0) < 1.0e-12);

eskf = propagate_inertial(eskf, inertial_sample(2.0, [0; 0; 0], [0; 0; 9.8066]));
assert(abs(eskf.state.last_inertial_stamp_sec - 2.0) < 1.0e-12);
assert(norm(eskf.state.position) < 1.0e-12);

bad_pose = pose_sample(2.01, [0; 0; NaN], [1 0 0 0]);
[ eskf, accepted, rejected ] = update_pose(eskf, bad_pose);
assert(~accepted && ~rejected);
result = eskf.state.position;
end

    function result = scenario_bias_and_corrected_pose(cfg) cfg.pose_position_noise_std = 0.01;
cfg.pose_orientation_noise_std = 0.01;
cfg.gyro_bias_random_walk_std = 1.0e-4;
eskf = reset_eskf(cfg);
eskf = initialize_from_pose(eskf, pose_sample(1.0, [0; 0; 0], [1 0 0 0]),
                            ... inertial_sample(1.0, [0; 0; 0], [0; 0; 9.8066]));

yaw_pose = pose_sample(1.1, [0.2; 0; 0], rpy_to_quat([0; 0; 0.1]));
[ eskf, accepted, ~] = update_pose(eskf, yaw_pose);
assert(accepted);
assert(abs(eskf.corrected_body_pose.position(1) - 0.2) < 1.0e-12);
assert(eskf.state.position(1) > 0.0);
assert(trace(eskf.covariance) > 0.0);
result = eskf.state.position;
end

    function cfg = default_config() cfg.gravity_mps2 = 9.8066;
cfg.measurement_frame_to_world = identity_pose();
cfg.body_to_marker = identity_pose();
cfg.estimate_extrinsic = false;
cfg.accel_noise_std = 0.35;
cfg.gyro_noise_std = 0.03;
cfg.pose_position_noise_std = 0.01;
cfg.pose_orientation_noise_std = 0.01;
cfg.gyro_bias_random_walk_std = 1.0e-4;
cfg.accel_bias_random_walk_std = 1.0e-3;
cfg.extrinsic_position_random_walk_std = 1.0e-5;
cfg.extrinsic_orientation_random_walk_std = 1.0e-5;
cfg.innovation_position_gate_m = 1.5;
cfg.innovation_orientation_gate_rad = 0.8;
cfg.max_propagation_dt_s = 0.05;
cfg.initial_position_variance = 0.01;
cfg.initial_velocity_variance = 0.1;
cfg.initial_orientation_variance = 0.01;
cfg.initial_gyro_bias_variance = 0.01;
cfg.initial_accel_bias_variance = 0.1;
cfg.initial_extrinsic_position_variance = 1.0e-12;
cfg.initial_extrinsic_orientation_variance = 1.0e-12;
end

    function eskf = reset_eskf(cfg) eskf.cfg = cfg;
eskf.state.initialized = false;
eskf.state.position = [0; 0; 0];
eskf.state.velocity = [0; 0; 0];
eskf.state.orientation = [1 0 0 0];
eskf.state.angular_velocity = [0; 0; 0];
eskf.state.gyro_bias = [0; 0; 0];
eskf.state.accel_bias = [0; 0; 0];
eskf.state.gravity = [0; 0; - cfg.gravity_mps2];
eskf.state.body_to_marker = cfg.body_to_marker;
eskf.state.last_inertial_stamp_sec = 0.0;
eskf.state.last_pose_stamp_sec = 0.0;
eskf.covariance = reset_covariance(cfg);
eskf.state.covariance_trace = trace(eskf.covariance);
eskf.corrected_body_pose = identity_pose();
eskf.has_corrected_body_pose = false;
end

    function eskf = initialize_from_pose(eskf, pose, imu) if ~valid_pose(pose) return;
end marker_world = pose_compose(eskf.cfg.measurement_frame_to_world, pose.pose);
body_world = pose_compose(marker_world, pose_inverse(eskf.cfg.body_to_marker));
eskf.covariance = reset_covariance(eskf.cfg);
eskf.state.position = body_world.position;
eskf.state.velocity = [0; 0; 0];
eskf.state.orientation = quat_normalize(body_world.orientation);
eskf.state.gravity = [0; 0; - eskf.cfg.gravity_mps2];
eskf.state.body_to_marker = eskf.cfg.body_to_marker;
eskf.state.last_pose_stamp_sec = pose.stamp_sec;
if valid_inertial (imu)
    eskf.state.angular_velocity = imu.angular_velocity - eskf.state.gyro_bias;
eskf.state.last_inertial_stamp_sec = imu.stamp_sec;
end eskf.state.covariance_trace = trace(eskf.covariance);
eskf.state.initialized = true;
eskf.corrected_body_pose = body_world;
eskf.has_corrected_body_pose = true;
end

    function eskf = propagate_inertial(eskf, imu) if ~valid_inertial(imu) return;
end eskf.state.angular_velocity = imu.angular_velocity - eskf.state.gyro_bias;
if
    ~eskf.state.initialized eskf.state.last_inertial_stamp_sec = imu.stamp_sec;
return;
end dt = imu.stamp_sec - eskf.state.last_inertial_stamp_sec;
if
    ~isfinite(dt) || dt <= 1.0e-5 eskf.state.last_inertial_stamp_sec = imu.stamp_sec;
return;
end if dt > eskf.cfg.max_propagation_dt_s eskf.state.last_inertial_stamp_sec = imu.stamp_sec;
eskf = inflate_covariance(eskf, eskf.cfg.max_propagation_dt_s);
return;
end specific_force = imu.linear_acceleration - eskf.state.accel_bias;
rotation = quat_to_rot(eskf.state.orientation);
accel_world = rotation * specific_force + eskf.state.gravity;
eskf.state.position = eskf.state.position + eskf.state.velocity * dt + 0.5 * accel_world * dt * dt;
eskf.state.velocity = eskf.state.velocity + accel_world * dt;
eskf.state.orientation = quat_normalize(quat_mul(eskf.state.orientation, quat_exp(eskf.state.angular_velocity* dt)));
eskf.state.last_inertial_stamp_sec = imu.stamp_sec;
eskf = propagate_covariance(eskf, specific_force, rotation, dt);
end

    function[eskf, accepted, rejected] = update_pose(eskf, pose) accepted = false;
rejected = false;
if
    ~valid_pose(pose) return;
end marker_world = pose_compose(eskf.cfg.measurement_frame_to_world, pose.pose);
if
    ~eskf.state.initialized eskf.corrected_body_pose =
        pose_compose(marker_world, pose_inverse(eskf.cfg.body_to_marker));
eskf.has_corrected_body_pose = true;
return;
end predicted_marker = predicted_marker_from_state(eskf.state);
innovation = measurement_residual(predicted_marker, marker_world);
if norm (innovation(1 : 3))
    > eskf.cfg.innovation_position_gate_m ||
        ... norm(innovation(4 : 6)) > eskf.cfg.innovation_orientation_gate_rad rejected = true;
eskf.state.last_pose_stamp_sec = pose.stamp_sec;
return;
end H = measurement_jacobian(eskf, predicted_marker, marker_world);
R = measurement_covariance(eskf.cfg);
S = H * eskf.covariance * H' + R; if rcond (S) < 1.0e-14 return;
end K = eskf.covariance* H' / S; delta = K * innovation;
eskf.state = inject_error_state(eskf.state, delta, eskf.cfg);
I = eye(21);
eskf.covariance = (I - K * H) * eskf.covariance * (I - K * H)' + K * R * K';
eskf.covariance = 0.5 * (eskf.covariance + eskf.covariance');
eskf.state.last_pose_stamp_sec = pose.stamp_sec;
eskf.corrected_body_pose = pose_compose(marker_world, pose_inverse(eskf.state.body_to_marker));
eskf.has_corrected_body_pose = true;
eskf.state.covariance_trace = trace(eskf.covariance);
accepted = true;
end

function P = reset_covariance(cfg)
P = eye(21) * 1.0e-6;
P(1:3, 1:3) = eye(3) * cfg.initial_position_variance;
P(4:6, 4:6) = eye(3) * cfg.initial_velocity_variance;
P(7:9, 7:9) = eye(3) * cfg.initial_orientation_variance;
P(10:12, 10:12) = eye(3) * cfg.initial_gyro_bias_variance;
P(13:15, 13:15) = eye(3) * cfg.initial_accel_bias_variance;
P(16:18, 16:18) = eye(3) * cfg.initial_extrinsic_position_variance;
P(19:21, 19:21) = eye(3) * cfg.initial_extrinsic_orientation_variance;
end

function eskf = propagate_covariance(eskf, accel_body, rotation, dt)
F = eye(21);
F(1:3, 4:6) = eye(3) * dt;
F(1:3, 7:9) = -0.5 * rotation * skew3(accel_body) * dt * dt;
F(1:3, 13:15) = -0.5 * rotation * dt * dt;
F(4:6, 7:9) = -rotation * skew3(accel_body) * dt;
F(4:6, 13:15) = -rotation * dt;
F(7:9, 10:12) = -eye(3) * dt;

Q = zeros(21);
Q(1:3, 1:3) = eye(3) * 0.25 * eskf.cfg.accel_noise_std^2 * dt^4;
Q(4:6, 4:6) = eye(3) * eskf.cfg.accel_noise_std^2 * dt^2;
Q(7:9, 7:9) = eye(3) * eskf.cfg.gyro_noise_std^2 * dt^2;
Q(10:12, 10:12) = eye(3) * eskf.cfg.gyro_bias_random_walk_std^2 * dt;
Q(13:15, 13:15) = eye(3) * eskf.cfg.accel_bias_random_walk_std^2 * dt;
if eskf.cfg.estimate_extrinsic
    Q(16:18, 16:18) = eye(3) * eskf.cfg.extrinsic_position_random_walk_std^2 * dt;
    Q(19:21, 19:21) = eye(3) * eskf.cfg.extrinsic_orientation_random_walk_std^2 * dt;
end
eskf.covariance = F * eskf.covariance * F' + Q;
eskf.covariance = 0.5 * (eskf.covariance + eskf.covariance');
eskf.state.covariance_trace = trace(eskf.covariance);
end

function eskf = inflate_covariance(eskf, dt)
Q = zeros(21);
Q(1:3, 1:3) = eye(3) * eskf.cfg.accel_noise_std^2 * dt * dt;
Q(4:6, 4:6) = eye(3) * eskf.cfg.accel_noise_std^2 * dt;
Q(7:9, 7:9) = eye(3) * eskf.cfg.gyro_noise_std^2 * dt;
eskf.covariance = eskf.covariance + Q;
eskf.covariance = 0.5 * (eskf.covariance + eskf.covariance');
eskf.state.covariance_trace = trace(eskf.covariance);
end

function H = measurement_jacobian(eskf, predicted_marker, measured_marker)
H = zeros(6, 21);
nominal = measurement_residual(predicted_marker, measured_marker);
for i = 1:21
    delta = zeros(21, 1);
    delta(i) = 1.0e-6;
    perturbed = inject_error_state(eskf.state, delta, eskf.cfg);
    perturbed_residual = measurement_residual(predicted_marker_from_state(perturbed), measured_marker);
    H(:, i) = -(perturbed_residual - nominal) / 1.0e-6;
end
if ~eskf.cfg.estimate_extrinsic
    H(:, 16:21) = 0.0;
end
end

function R = measurement_covariance(cfg)
R = zeros(6);
R(1:3, 1:3) = eye(3) * cfg.pose_position_noise_std^2;
R(4:6, 4:6) = eye(3) * cfg.pose_orientation_noise_std^2;
end

function state = inject_error_state(state, delta, cfg)
state.position = state.position + delta(1:3);
state.velocity = state.velocity + delta(4:6);
state.orientation = quat_normalize(quat_mul(state.orientation, quat_exp(delta(7:9))));
state.gyro_bias = state.gyro_bias + delta(10:12);
state.accel_bias = state.accel_bias + delta(13:15);
if cfg.estimate_extrinsic
    state.body_to_marker.position = state.body_to_marker.position + delta(16:18);
    state.body_to_marker.orientation = quat_normalize( ...
        quat_mul(state.body_to_marker.orientation, quat_exp(delta(19:21))));
end
end

function predicted_marker = predicted_marker_from_state(state)
body_pose.position = state.position;
body_pose.orientation = state.orientation;
predicted_marker = pose_compose(body_pose, state.body_to_marker);
end

function r = measurement_residual(predicted_marker, measured_marker)
r = se3_error(predicted_marker, measured_marker);
end

function r = se3_error(reference, measured)
delta = pose_compose(pose_inverse(reference), measured);
r = [delta.position; quat_log(delta.orientation)];
end

function sample = inertial_sample(stamp_sec, gyro, accel)
sample.received = true;
sample.valid = true;
sample.stamp_sec = stamp_sec;
sample.angular_velocity = gyro;
sample.linear_acceleration = accel;
end

function sample = pose_sample(stamp_sec, position, orientation)
sample.received = true;
sample.valid = true;
sample.stamp_sec = stamp_sec;
sample.pose.position = position;
sample.pose.orientation = quat_normalize(orientation);
end

function ok = valid_inertial(sample)
ok = sample.received && sample.valid && isfinite(sample.stamp_sec) && ...
    all(isfinite(sample.angular_velocity)) && all(isfinite(sample.linear_acceleration));
end

function ok = valid_pose(sample)
ok = sample.received && sample.valid && isfinite(sample.stamp_sec) && ...
    all(isfinite(sample.pose.position)) && all(isfinite(sample.pose.orientation));
end

function T = identity_pose()
T.position = [0; 0; 0];
T.orientation = [1 0 0 0];
end

function T = pose_compose(A, B)
T.position = A.position + quat_rotate(A.orientation, B.position);
T.orientation = quat_normalize(quat_mul(A.orientation, B.orientation));
end

function T = pose_inverse(A)
T.orientation = quat_conj(quat_normalize(A.orientation));
T.position = -quat_rotate(T.orientation, A.position);
end

function q = rpy_to_quat(rpy)
cr = cos(rpy(1) / 2); sr = sin(rpy(1) / 2);
cp = cos(rpy(2) / 2); sp = sin(rpy(2) / 2);
cy = cos(rpy(3) / 2); sy = sin(rpy(3) / 2);
q = quat_normalize([ ...
    cy * cp * cr + sy * sp * sr, ...
    cy * cp * sr - sy * sp * cr, ...
    sy * cp * sr + cy * sp * cr, ...
    sy * cp * cr - cy * sp * sr]);
end

function q = quat_normalize(q)
if any(~isfinite(q)) || norm(q) <= 1.0e-12
    q = [1 0 0 0];
    return;
end
q = q / norm(q);
if q(1) < 0
    q = -q;
end
end

function q = quat_mul(a, b)
q = [ ...
    a(1) * b(1) - a(2) * b(2) - a(3) * b(3) - a(4) * b(4), ...
    a(1) * b(2) + a(2) * b(1) + a(3) * b(4) - a(4) * b(3), ...
    a(1) * b(3) - a(2) * b(4) + a(3) * b(1) + a(4) * b(2), ...
    a(1) * b(4) + a(2) * b(3) - a(3) * b(2) + a(4) * b(1)];
end

function q = quat_conj(q)
q = [q(1) -q(2) -q(3) -q(4)];
end

function v = quat_rotate(q, v)
q = quat_normalize(q);
vq = quat_mul(quat_mul(q, [0 v(:)']), quat_conj(q));
v = vq(2:4)';
end

function R = quat_to_rot(q)
q = quat_normalize(q);
w = q(1); x = q(2); y = q(3); z = q(4);
R = [ ...
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w); ...
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w); ...
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)];
end

function q = quat_exp(rotvec)
angle = norm(rotvec);
if ~isfinite(angle) || angle <= 1.0e-12
    q = [1 0 0 0];
    return;
end
axis = rotvec(:)' / angle;
q = quat_normalize([cos(angle / 2) sin(angle / 2) * axis]);
end

function rotvec = quat_log(q)
q = quat_normalize(q);
v = q(2:4)';
sin_half = norm(v);
if sin_half <= 1.0e-12
    rotvec = [0; 0; 0];
    return;
end
angle = 2.0 * atan2(sin_half, q(1));
rotvec = angle * v / sin_half;
end

function y = clamp_norm(x, max_norm)
if any(~isfinite(x)) || ~isfinite(max_norm) || max_norm <= 0.0
    y = [0; 0; 0];
    return;
end
n = norm(x);
if n <= max_norm
    y = x;
else
    y = x * (max_norm / n);
end
end

function S = skew3(x)
S = [0, -x(3), x(2); x(3), 0, -x(1); -x(2), x(1), 0];
end
