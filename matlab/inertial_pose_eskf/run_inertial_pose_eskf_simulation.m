function results = run_inertial_pose_eskf_simulation()
%RUN_INERTIAL_POSE_ESKF_SIMULATION Reference scenarios for InertialPoseEskf.
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

function result = scenario_se3()
q = quat_normalize([-2.0 0.1 -0.2 0.3]);
assert(abs(norm(q) - 1.0) < 1.0e-12);
assert(q(1) >= 0.0);

T.position = [1.0; 2.0; 3.0];
T.orientation = rpy_to_quat([0.1; -0.2; 0.3]);
I = pose_compose(T, pose_inverse(T));
assert(norm(I.position) < 1.0e-12);
assert(abs(I.orientation(1) - 1.0) < 1.0e-12);
result = true;
end

function result = scenario_initialization(cfg)
cfg.measurement_frame_to_world.position = [1.0; 2.0; 3.0];
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

function result = scenario_stationary(cfg)
eskf = reset_eskf(cfg);
imu0 = inertial_sample(1.0, [0.1; 0; 0], [0; 0; 9.8066]);
eskf = initialize_from_pose(eskf, pose_sample(1.0, [0; 0; 0], [1 0 0 0]), imu0);
eskf = propagate_inertial(eskf, inertial_sample(1.01, [0.1; 0; 0], [0; 0; 9.8066]));

assert(abs(eskf.state.angular_velocity(1) - 0.1) < 1.0e-12);
assert(norm(eskf.state.velocity) < 1.0e-3);
assert(abs(norm(eskf.state.orientation) - 1.0) < 1.0e-12);
result = eskf.state.velocity;
end

function result = scenario_pose_update_and_reject(cfg)
cfg.innovation_position_gate_m = 0.5;
eskf = reset_eskf(cfg);
eskf = initialize_from_pose(eskf, pose_sample(1.0, [0; 0; 0], [1 0 0 0]), ...
    inertial_sample(1.0, [0; 0; 0], [0; 0; 9.8066]));

[eskf, accepted, rejected] = update_pose(eskf, pose_sample(1.02, [0.4; 0; 0], [1 0 0 0]));
assert(accepted && ~rejected);
assert(eskf.state.position(1) > 0.3 && eskf.state.position(1) < 0.4);
held_position = eskf.state.position(1);

[eskf, accepted, rejected] = update_pose(eskf, pose_sample(1.04, [2.0; 0; 0], [1 0 0 0]));
assert(~accepted && rejected);
assert(abs(eskf.state.position(1) - held_position) < 1.0e-12);
result = eskf.state.position;
end

function result = scenario_invalid_and_large_dt(cfg)
eskf = reset_eskf(cfg);
eskf = initialize_from_pose(eskf, pose_sample(1.0, [0; 0; 0], [1 0 0 0]), ...
    inertial_sample(1.0, [0; 0; 0], [0; 0; 9.8066]));

bad_imu = inertial_sample(1.01, [NaN; 0; 0], [0; 0; 9.8066]);
eskf = propagate_inertial(eskf, bad_imu);
assert(abs(eskf.state.last_inertial_stamp_sec - 1.0) < 1.0e-12);

eskf = propagate_inertial(eskf, inertial_sample(2.0, [0; 0; 0], [0; 0; 9.8066]));
assert(abs(eskf.state.last_inertial_stamp_sec - 2.0) < 1.0e-12);
assert(norm(eskf.state.position) < 1.0e-12);

bad_pose = pose_sample(2.01, [0; 0; NaN], [1 0 0 0]);
[eskf, accepted, rejected] = update_pose(eskf, bad_pose);
assert(~accepted && ~rejected);
result = eskf.state.position;
end

function result = scenario_bias_and_corrected_pose(cfg)
cfg.position_update_gain = 0.0;
cfg.velocity_update_gain = 0.0;
cfg.orientation_update_gain = 0.5;
cfg.gyro_bias_update_gain = 0.01;
eskf = reset_eskf(cfg);
eskf = initialize_from_pose(eskf, pose_sample(1.0, [0; 0; 0], [1 0 0 0]), ...
    inertial_sample(1.0, [0; 0; 0], [0; 0; 9.8066]));

yaw_pose = pose_sample(1.1, [0.2; 0; 0], rpy_to_quat([0; 0; 0.1]));
[eskf, accepted, ~] = update_pose(eskf, yaw_pose);
assert(accepted);
assert(abs(eskf.corrected_body_pose.position(1) - 0.2) < 1.0e-12);
assert(abs(eskf.state.position(1)) < 1.0e-12);
assert(eskf.state.gyro_bias(3) < 0.0);
result = eskf.state.gyro_bias;
end

function cfg = default_config()
cfg.gravity_mps2 = 9.8066;
cfg.measurement_frame_to_world = identity_pose();
cfg.body_to_marker = identity_pose();
cfg.accel_noise_std = 0.35;
cfg.gyro_noise_std = 0.03;
cfg.position_update_gain = 0.85;
cfg.velocity_update_gain = 0.25;
cfg.orientation_update_gain = 0.85;
cfg.gyro_bias_update_gain = 0.002;
cfg.innovation_position_gate_m = 1.5;
cfg.innovation_orientation_gate_rad = 0.8;
end

function eskf = reset_eskf(cfg)
eskf.cfg = cfg;
eskf.state.initialized = false;
eskf.state.position = [0; 0; 0];
eskf.state.velocity = [0; 0; 0];
eskf.state.orientation = [1 0 0 0];
eskf.state.angular_velocity = [0; 0; 0];
eskf.state.gyro_bias = [0; 0; 0];
eskf.state.accel_bias = [0; 0; 0];
eskf.state.gravity = [0; 0; -cfg.gravity_mps2];
eskf.state.body_to_marker = cfg.body_to_marker;
eskf.state.last_inertial_stamp_sec = 0.0;
eskf.state.last_pose_stamp_sec = 0.0;
eskf.state.covariance_trace = 1.0;
eskf.corrected_body_pose = identity_pose();
eskf.has_corrected_body_pose = false;
end

function eskf = initialize_from_pose(eskf, pose, imu)
if ~valid_pose(pose)
    return;
end
marker_world = pose_compose(eskf.cfg.measurement_frame_to_world, pose.pose);
body_world = pose_compose(marker_world, pose_inverse(eskf.cfg.body_to_marker));
eskf.state.position = body_world.position;
eskf.state.velocity = [0; 0; 0];
eskf.state.orientation = quat_normalize(body_world.orientation);
eskf.state.last_pose_stamp_sec = pose.stamp_sec;
if valid_inertial(imu)
    eskf.state.angular_velocity = imu.angular_velocity - eskf.state.gyro_bias;
    eskf.state.last_inertial_stamp_sec = imu.stamp_sec;
end
eskf.state.covariance_trace = 0.1;
eskf.state.initialized = true;
eskf.corrected_body_pose = body_world;
eskf.has_corrected_body_pose = true;
end

function eskf = propagate_inertial(eskf, imu)
if ~valid_inertial(imu)
    return;
end
eskf.state.angular_velocity = imu.angular_velocity - eskf.state.gyro_bias;
if ~eskf.state.initialized
    eskf.state.last_inertial_stamp_sec = imu.stamp_sec;
    return;
end
dt = imu.stamp_sec - eskf.state.last_inertial_stamp_sec;
if ~isfinite(dt) || dt <= 1.0e-5 || dt > 0.05
    eskf.state.last_inertial_stamp_sec = imu.stamp_sec;
    return;
end
dq = quat_exp(eskf.state.angular_velocity * dt);
eskf.state.orientation = quat_normalize(quat_mul(eskf.state.orientation, dq));
specific_force = imu.linear_acceleration - eskf.state.accel_bias;
accel_world = quat_rotate(eskf.state.orientation, specific_force) + eskf.state.gravity;
eskf.state.position = eskf.state.position + eskf.state.velocity * dt + 0.5 * accel_world * dt * dt;
eskf.state.velocity = eskf.state.velocity + accel_world * dt;
eskf.state.last_inertial_stamp_sec = imu.stamp_sec;
eskf.state.covariance_trace = eskf.state.covariance_trace + ...
    max(0.0, dt) * (eskf.cfg.accel_noise_std^2 + eskf.cfg.gyro_noise_std^2);
end

function [eskf, accepted, rejected] = update_pose(eskf, pose)
accepted = false;
rejected = false;
if ~valid_pose(pose)
    return;
end
marker_world = pose_compose(eskf.cfg.measurement_frame_to_world, pose.pose);
corrected_body_pose = pose_compose(marker_world, pose_inverse(eskf.cfg.body_to_marker));
if ~eskf.state.initialized
    eskf.corrected_body_pose = corrected_body_pose;
    eskf.has_corrected_body_pose = true;
    return;
end
predicted_body.position = eskf.state.position;
predicted_body.orientation = eskf.state.orientation;
predicted_marker = pose_compose(predicted_body, eskf.cfg.body_to_marker);
position_residual = marker_world.position - predicted_marker.position;
orientation_residual = quat_normalize(quat_mul(quat_conj(predicted_marker.orientation), marker_world.orientation));
rotation_residual = quat_log(orientation_residual);
if norm(position_residual) > eskf.cfg.innovation_position_gate_m || ...
        norm(rotation_residual) > eskf.cfg.innovation_orientation_gate_rad
    rejected = true;
    eskf.state.last_pose_stamp_sec = pose.stamp_sec;
    return;
end
dt_pose = pose.stamp_sec - eskf.state.last_pose_stamp_sec;
if isfinite(dt_pose) && dt_pose > 1.0e-5
    velocity_dt = dt_pose;
else
    velocity_dt = 0.0;
end
eskf.state.position = eskf.state.position + eskf.cfg.position_update_gain * position_residual;
if velocity_dt > 0.0
    eskf.state.velocity = eskf.state.velocity + eskf.cfg.velocity_update_gain * ...
        (position_residual / max(velocity_dt, 0.01));
end
eskf.state.orientation = quat_normalize(quat_mul(eskf.state.orientation, ...
    quat_exp(eskf.cfg.orientation_update_gain * rotation_residual)));
if velocity_dt > 0.0 && eskf.cfg.gyro_bias_update_gain > 0.0
    bias_step = clamp_norm(-eskf.cfg.gyro_bias_update_gain * rotation_residual / velocity_dt, 0.01);
    eskf.state.gyro_bias = eskf.state.gyro_bias + bias_step;
end
eskf.state.last_pose_stamp_sec = pose.stamp_sec;
eskf.corrected_body_pose = corrected_body_pose;
eskf.has_corrected_body_pose = true;
eskf.state.covariance_trace = max(1.0e-4, eskf.state.covariance_trace * 0.5);
accepted = true;
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
