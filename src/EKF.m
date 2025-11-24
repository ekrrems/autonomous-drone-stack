%%% =========================================================
%   IMU-only EKF with Ground Truth Position Updates (UZH-FPV)
%   - World frame: ENU  (X-East, Y-North, Z-Up)
%   - IMU frame:  X-forward, Y-right, Z-down
%%% =========================================================

clc; clear; close all;

%% ---------------------------------------------------------
%   LOAD BAG AND EXTRACT DATA
% ----------------------------------------------------------
bagPath = fullfile(pwd, "../data/converted_bag/");

imgTopic = "/snappy_cam/stereo_l";
imuTopic = "/snappy_imu";
gtTopic  = "/groundtruth/pose";

bag = ros2bagreader(bagPath);

imgMsgs = readMessages(select(bag, 'Topic', imgTopic));
imuMsgs = readMessages(select(bag, 'Topic', imuTopic));
gtMsgs  = readMessages(select(bag, 'Topic', gtTopic));

fprintf("Loaded %d IMU  | %d GT | %d images\n", ...
        numel(imuMsgs), numel(gtMsgs), numel(imgMsgs));

%% ---------------------------------------------------------
%   TIMESTAMPS
% ---------------------------------------------------------
imuTimes = cellfun(@(m) double(m.header.stamp.sec) + ...
                           double(m.header.stamp.nanosec)*1e-9, imuMsgs);

gtTimes = cellfun(@(m) double(m.header.stamp.sec) + ...
                          double(m.header.stamp.nanosec)*1e-9, gtMsgs);

%% ---------------------------------------------------------
%   GROUND TRUTH POSITIONS (XYZ) + ORIENTATION  (world=ENU)
% ---------------------------------------------------------
gtPos = zeros(3, numel(gtMsgs));
gtRot = zeros(4, numel(gtMsgs));   % quaternion [w x y z]

for i = 1:numel(gtMsgs)
    gtPos(:,i) = [gtMsgs{i}.pose.position.x;
                  gtMsgs{i}.pose.position.y;
                  gtMsgs{i}.pose.position.z];

    q_tmp = [gtMsgs{i}.pose.orientation.w;
             gtMsgs{i}.pose.orientation.x;
             gtMsgs{i}.pose.orientation.y;
             gtMsgs{i}.pose.orientation.z];

    gtRot(:,i) = q_tmp / norm(q_tmp);
end

% Shift GT so trajectory starts at origin (only for plotting/comparison)
gt_pos0   = gtPos(:,1);
gtPos_rel = gtPos - gt_pos0;

% Interpolate GT to IMU timestamps
gt_interp = interp1(gtTimes, gtPos_rel', imuTimes, 'linear', 'extrap')';

%% ---------------------------------------------------------
%   IMU-BASED STATISTICS FOR INITIALIZATION
% ---------------------------------------------------------
N  = numel(imuMsgs);
N0 = min(200, N);   % first ~200 IMU samples

acc0  = zeros(3, N0);
gyro0 = zeros(3, N0);

for i = 1:N0
    a_body = [imuMsgs{i}.linear_acceleration.x;
              imuMsgs{i}.linear_acceleration.y;
              imuMsgs{i}.linear_acceleration.z];

    w_body = [imuMsgs{i}.angular_velocity.x;
              imuMsgs{i}.angular_velocity.y;
              imuMsgs{i}.angular_velocity.z];

    % transform IMU frame -> ENU
    acc0(:,i)  = imu2enu(a_body);
    gyro0(:,i) = imu2enu(w_body);
end

mean_acc  = mean(acc0, 2);   % ENU
mean_gyro = mean(gyro0, 2);  % ENU

%% ---------------------------------------------------------
%   INITIAL EKF STATE (world frame = ENU of /groundtruth/pose)
% ---------------------------------------------------------
state = struct();

% Use ground-truth orientation at first GT sample
q0 = gtRot(:,1);           % [w x y z]
q0 = q0 / norm(q0);

state.q = q0;              % orientation body->world (ENU)
state.p = zeros(3,1);      % position starts at origin (relative GT)
state.v = zeros(3,1);      % assume small initial velocity

% Gravity in ENU
g_enu = [0; 0; -9.81];

% Body->world rotation at t0
Rwb0 = quat2rotm_matlab(state.q);

% Choose accel bias so that a_world ≈ 0 on average in the "static" window:
% a_world = Rwb0 * (mean_acc - ba) + g_enu ≈ 0  ⇒  ba ≈ mean_acc + Rwb0' * g_enu
state.ba = mean_acc + Rwb0.' * g_enu;

% Gyro bias from mean
state.bg = mean_gyro;

% 15D error state covariance
P = 1e-4 * eye(15);

% Process noise std devs (tune if you want)
noise.sigma_a  = 0.02;    % accel noise  [m/s^2 / sqrt(Hz)]
noise.sigma_g  = 0.001;   % gyro noise   [rad/s / sqrt(Hz)]
noise.sigma_ba = 0.0001;  % accel bias rw
noise.sigma_bg = 0.0001;  % gyro bias rw

%% ---------------------------------------------------------
%   VISUALIZATION SETUP
% ---------------------------------------------------------
figure; hold on; grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal;
title("IMU-only EKF (blue) vs Ground Truth (green)");

traj_est = animatedline('Color','b','LineWidth',2); % EKF
traj_gt  = animatedline('Color','g','LineWidth',2); % Ground truth

% Plot full GT once (relative, in ENU)
addpoints(traj_gt, gtPos_rel(1,:), gtPos_rel(2,:), gtPos_rel(3,:));

%% ---------------------------------------------------------
%   MAIN EKF LOOP: Prediction (IMU) + Update (GT position)
% ---------------------------------------------------------
estPos = zeros(3, N);

R_meas = (0.05^2) * eye(3);   % 5 cm std-dev position measurement

for k = 2:N

    dt = imuTimes(k) - imuTimes(k-1);
    if dt <= 0, continue; end

    % --------------------------
    % RAW IMU (body frame, X fwd, Y right, Z down)
    % --------------------------
    acc_body = [imuMsgs{k}.linear_acceleration.x;
                imuMsgs{k}.linear_acceleration.y;
                imuMsgs{k}.linear_acceleration.z];

    gyro_body = [imuMsgs{k}.angular_velocity.x;
                 imuMsgs{k}.angular_velocity.y;
                 imuMsgs{k}.angular_velocity.z];

    % Transform IMU -> ENU frame
    imu.acc  = imu2enu(acc_body);
    imu.gyro = imu2enu(gyro_body);

    % -----------------------------------------------------
    % 1) EKF prediction (IMU only)
    % -----------------------------------------------------
    [state, P] = ekf_predict_imu(state, P, imu, dt, noise);

    % -----------------------------------------------------
    % 2) EKF update using GT position as "GPS" (for testing)
    % -----------------------------------------------------
    if k <= size(gt_interp,2)
        z = gt_interp(:,k);   % 3x1 position in ENU, relative
    else
        z = gt_interp(:,end);
    end

    [state, P] = ekf_update_position(state, P, z, R_meas);

    estPos(:,k) = state.p;

    % -----------------------------------------------------
    % 3) Plot
    % -----------------------------------------------------
    addpoints(traj_est, state.p(1), state.p(2), state.p(3));
    drawnow limitrate;
end

disp("Done.");

%% ================== Helper functions ==================

function v_enu = imu2enu(v)
    % IMU frame: x forward, y right, z down
    % ENU frame: x East, y North, z Up
    % Mapping:
    %   imu x (forward) -> ENU y
    %   imu y (right)   -> ENU x
    %   imu z (down)    -> ENU -z
    R = [ 0  1  0;
          1  0  0;
          0  0 -1 ];
    v_enu = R * v;
end

function S = skew(v)
    S = [   0    -v(3)  v(2);
           v(3)   0    -v(1);
          -v(2)  v(1)   0   ];
end

function R = quat2rotm_matlab(q)
    % q = [w x y z]
    w = q(1); x = q(2); y = q(3); z = q(4);
    R = [1-2*(y^2+z^2),   2*(x*y - z*w),   2*(x*z + y*w);
         2*(x*y + z*w),   1-2*(x^2+z^2),   2*(y*z - x*w);
         2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x^2+y^2)];
end

function q_out = quat_mult(q1, q2)
    % Hamilton product q_out = q1 ⊗ q2, both [w x y z]
    w1=q1(1); x1=q1(2); y1=q1(3); z1=q1(4);
    w2=q2(1); x2=q2(2); y2=q2(3); z2=q2(4);

    q_out = [ w1*w2 - x1*x2 - y1*y2 - z1*z2;
              w1*x2 + x1*w2 + y1*z2 - z1*y2;
              w1*y2 - x1*z2 + y1*w2 + z1*x2;
              w1*z2 + x1*y2 - y1*x2 + z1*w2 ];
end

function q = expmap_quat(dtheta)
    % Small rotation vector dtheta (3x1) -> quaternion [w x y z]
    theta = norm(dtheta);
    if theta < 1e-8
        q = [1; 0; 0; 0];
    else
        axis = dtheta / theta;
        half = 0.5 * theta;
        q = [cos(half);
             axis(1)*sin(half);
             axis(2)*sin(half);
             axis(3)*sin(half)];
    end
end

%% ================== EKF prediction ==================
function [state, P] = ekf_predict_imu(state, P, imu, dt, noise)
    % EKF IMU prediction (error-state, 15D)
    % state: struct with fields p,v,q,ba,bg
    % P    : 15x15 error covariance
    % imu  : struct with fields acc (3x1), gyro (3x1)
    % dt   : time step
    % noise: struct with sigma_a, sigma_g, sigma_ba, sigma_bg

    g = [0; 0; -9.81];   % gravity in world (ENU)

    % ----- 1) Nominal propagation -----
    a_m = imu.acc;   % measured accel (body)  (specific force)
    w_m = imu.gyro;  % measured gyro  (body)

    a = a_m - state.ba;   % bias corrected accel
    w = w_m - state.bg;   % bias corrected gyro

    % Rotation body -> world
    Rwb = quat2rotm_matlab(state.q);

    % world-frame acceleration
    acc_world = Rwb * a + g;

    state.p = state.p + state.v * dt + 0.5 * acc_world * dt^2;
    state.v = state.v + acc_world * dt;

    % Quaternion integration
    dq = expmap_quat(w * dt);
    state.q = quat_mult(state.q, dq);
    state.q = state.q / norm(state.q);

    % ----- 2) Error-state Jacobians F and G -----
    F = zeros(15,15);
    G = zeros(15,12);

    I3 = eye(3);

    % indices: [δp(1:3); δv(4:6); δθ(7:9); δba(10:12); δbg(13:15)]

    % δp_dot = δv
    F(1:3,4:6) = I3;

    % δv_dot = -R*[a]_x δθ - R δba
    F(4:6,7:9)   = -Rwb * skew(a);
    F(4:6,10:12) = -Rwb;

    % δθ_dot = -[w]_x δθ - δbg
    F(7:9,7:9)   = -skew(w);
    F(7:9,13:15) = -I3;

    % Noise mapping (process noise = [n_a; n_g; n_ba; n_bg] ∈ R^12)
    G(4:6,1:3)      = -Rwb;   % accel noise
    G(7:9,4:6)      = -I3;    % gyro noise
    G(10:12,7:9)    = I3;     % accel bias rw
    G(13:15,10:12)  = I3;     % gyro bias rw

    % ----- 3) Discretize -----
    Phi = eye(15) + F*dt;   % first-order state transition

    Qa   = (noise.sigma_a)^2   * eye(3);
    Qg   = (noise.sigma_g)^2   * eye(3);
    Qba  = (noise.sigma_ba)^2  * eye(3);
    Qbg  = (noise.sigma_bg)^2  * eye(3);

    Qc = blkdiag(Qa, Qg, Qba, Qbg);  % 12x12 continuous

    Qd = G * Qc * G.' * dt;          % discrete approx

    % ----- 4) Covariance propagation -----
    P = Phi * P * Phi.' + Qd;
end

%% ================== EKF position-only update ==================
function [state, P] = ekf_update_position(state, P, z, R)
    % Position-only measurement update for error-state EKF
    % state: struct with fields p,v,q,ba,bg
    % P    : 15x15 covariance
    % z    : 3x1 measured position (e.g. GT)
    % R    : 3x3 measurement noise covariance

    I3  = eye(3);
    I15 = eye(15);

    % measurement jacobian H (3x15)
    H = zeros(3,15);
    H(1:3,1:3) = I3;   % z = p + noise

    % innovation
    z_pred = state.p;
    y = z - z_pred;

    % Kalman gain
    S = H * P * H.' + R;
    K = P * H.' / S;    % 15x3

    % error-state update
    delta_x = K * y;    % 15x1

    delta_p  = delta_x(1:3);
    delta_v  = delta_x(4:6);
    delta_th = delta_x(7:9);
    delta_ba = delta_x(10:12);
    delta_bg = delta_x(13:15);

    state.p  = state.p  + delta_p;
    state.v  = state.v  + delta_v;
    dq       = expmap_quat(delta_th);
    state.q  = quat_mult(dq, state.q);
    state.q  = state.q / norm(state.q);
    state.ba = state.ba + delta_ba;
    state.bg = state.bg + delta_bg;

    % covariance update (Joseph form)
    P = (I15 - K*H) * P * (I15 - K*H).' + K*R*K.';
end