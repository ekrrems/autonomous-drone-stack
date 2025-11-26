%%% =========================================================
%   Real-time IMU + OpenVINS Transformation Fusion
%%% =========================================================

clc; clear; close all;

%% ----------------- GLOBAL STATE (shared with callbacks) ---
global ekf_state ekf_P ekf_initialized last_imu_time ekf_noise;
global traj_est ovins_line;

ekf_initialized = false;
last_imu_time   = [];

% allocate state & covariance (values will be overwritten on init)
ekf_state = struct( ...
    'p',  zeros(3,1), ...       % position
    'v',  zeros(3,1), ...       % velocity
    'q',  [1;0;0;0], ...        % quaternion body->world [w x y z]
    'ba', zeros(3,1), ...       % accel bias
    'bg', zeros(3,1));          % gyro bias

ekf_P = 1e-4 * eye(15);         % 15D error covariance

% Process-noise parameters
ekf_noise.sigma_a  = 0.05;      % m/s^2 / sqrt(Hz)
ekf_noise.sigma_g  = 0.005;     % rad/s / sqrt(Hz)
ekf_noise.sigma_ba = 0.0005;    % accel bias Random Walk (RW)
ekf_noise.sigma_bg = 0.0005;    % gyro  bias RW

%% Initiate Figure
figure; hold on; grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal;
title('IMU+OVINS EKF  (blue)  vs  OVINS pose (red)');

traj_est  = animatedline('Color','b','LineWidth',2); % EKF fused traj
ovins_line = animatedline('Color','r','LineWidth',2); % raw OVINS pose

view(2);   % XY Plane

%% TCP Connection Setup
port = 5055;
server = tcpserver("0.0.0.0", port, ...
    "ConnectionChangedFcn", @connectionHandler);

disp("Waiting for TCP client (Docker / ESP32 / etc.) ...");

while true
    pause(0.5);
    if server.Connected
        disp("Client connected!");
        break;
    end
end

% Callback for whenever bytes arrive
configureCallback(server, "byte", 1, @dataReceived);
disp("Callbacks configured. EKF now running event-driven.");

while true        % Without this the script gets stuck at callbacks
    pause(0.1);
end

%% Callbacks & Data Parsers

function dataReceived(server, ~)
    % Called whenever new bytes arrive on the TCP socket
    persistent buffer;
    if isempty(buffer)
        buffer = uint8([]);
    end

    if server.NumBytesAvailable <= 0
        return;
    end

    newBytes = read(server, server.NumBytesAvailable, "uint8");
    buffer   = [buffer; newBytes(:)];

    buffer = parseMessages(buffer);
end

function buffer = parseMessages(buffer)
    while true
        try
            if numel(buffer) < 7
                break;                       % not enough for header+len
            end

            header = char(buffer(1:3).');    % 'IMU','PTH',...
            payloadLen = typecast(uint8(buffer(4:7)), 'uint32');

            if numel(buffer) < 7 + payloadLen
                break;                       % wait for full message
            end

            payload = buffer(8 : 7+payloadLen);
            buffer  = buffer(8+payloadLen : end);  % drop processed

            switch header
                case "IMU"
                    handleIMU(payload);
                case "PTH"
                    handlePTH(payload);
                case "PCD"
                    % Could be used to show 3D points
                case "IMG"
                    % Could be matched with the poses
                otherwise
                    fprintf("❓ Unknown header: %s\n", header);
            end

        catch ME
            fprintf("❌ parseMessages error: %s\n", ME.message);
            return;
        end
    end
end

function handleIMU(payload)
    % Parse IMU info and predict system state
    global ekf_state ekf_P ekf_initialized last_imu_time ekf_noise;

    try
        json_str = char(payload(:).');
        data = jsondecode(json_str);

        % ========= ADAPT TO YOUR JSON FORMAT HERE =========
        % Assumed JSON (example):
        % { "timestamp": 123.456,
        %   "ax": ..., "ay": ..., "az": ...,
        %   "gx": ..., "gy": ..., "gz": ... }
        t  = data.timestamp;     % [s]
        a_body = [data.ax; data.ay; data.az];   % m/s^2 (body frame)
        w_body = [data.gx; data.gy; data.gz];   % rad/s  (body frame)
        % ===================================================

        % First IMU → initialize simple state (flat, zero biases)
        if ~ekf_initialized
            ekf_state.p  = zeros(3,1);
            ekf_state.v  = zeros(3,1);
            ekf_state.q  = [1;0;0;0];   % identity: world~=body initially
            ekf_state.ba = zeros(3,1);
            ekf_state.bg = zeros(3,1);

            last_imu_time = t;
            ekf_initialized = true;
            fprintf("EKF initialized from first IMU sample.\n");
            return;
        end

        dt = t - last_imu_time;
        if dt <= 0
            return;
        end
        last_imu_time = t;

        % Convert IMU -> ENU
        imu.acc  = imu2enu(a_body);
        imu.gyro = imu2enu(w_body);

        [ekf_state, ekf_P] = ekf_predict_imu(ekf_state, ekf_P, imu, dt, ekf_noise);

        % uncomment to see pure prediction trajectory
        % global traj_est;
        % addpoints(traj_est, ekf_state.p(1), ekf_state.p(2), ekf_state.p(3));
        % drawnow limitrate;

    catch err
        fprintf("Error in handleIMU: %s\n", err.message);
    end
end

function handlePTH(payload)
    % Parse the OpenVINS Pose data and plot it
    global ekf_state ekf_P traj_est ovins_line;

    try
        json_str = char(payload(:).');
        data     = jsondecode(json_str);

        % Convert OVINS coordinate system to ENU
        R_ovins_to_enu = [ 0  1  0;
                          -1  0  0;
                           0  0  1];

        p_imu = [data.poses.x; data.poses.y; data.poses.z];
        p_enu = R_ovins_to_enu * p_imu;

        % Plot raw OVINS pose (red)
        addpoints(ovins_line, p_enu(1), p_enu(2), p_enu(3));

        % EKF update with this pose
        if ~isempty(ekf_P)
            R_meas = (0.02^2) * eye(3);   % 5 cm std-dev (Could be tuned)
            [ekf_state, ekf_P] = ekf_update_position(ekf_state, ekf_P, p_enu, R_meas);

            addpoints(traj_est, ekf_state.p(1), ekf_state.p(2), ekf_state.p(3));
            drawnow limitrate;
        end

    catch err
        fprintf("Error in handlePTH: %s\n", err.message);
    end
end

function connectionHandler(server, event)
    if server.Connected
        disp("✅ Client connected to MATLAB TCP server.");
    else
        disp("⚠️ Client disconnected.");
    end
end

%% ================== EKF & MATH HELPERS ====================

function v_enu = imu2enu(v)
    % IMU frame: x forward, y right, z down
    % ENU frame: x East, y North, z Up
    R = [ 0  1  0;    % x_fwd → y
          1  0  0;    % y_right → x
          0  0 -1 ];  % z_down → -z
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
    % Rotation vector dtheta (3x1) -> quaternion [w x y z]
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

function [state, P] = ekf_predict_imu(state, P, imu, dt, noise)
    % Error-state IMU prediction (15-dim error state).
    g = [0; 0; -9.81];   % gravity in world (ENU)

    % Nominal State variables
    a_m = imu.acc;
    w_m = imu.gyro;

    a = a_m - state.ba;
    w = w_m - state.bg;

    Rwb = quat2rotm_matlab(state.q);
    acc_world = Rwb * a + g;

    state.p = state.p + state.v * dt + 0.5 * acc_world * dt^2;
    state.v = state.v + acc_world * dt;

    dq = expmap_quat(w * dt);
    state.q = quat_mult(state.q, dq);
    state.q = state.q / norm(state.q);

    % Error State variables
    F = zeros(15,15);
    G = zeros(15,12);
    I3 = eye(3);

    % indices: [δp; δv; δθ; δba; δbg]

    F(1:3,4:6)   = I3;                 % δp_dot = δv
    F(4:6,7:9)   = -Rwb * skew(a);     % δv_dot part
    F(4:6,10:12) = -Rwb;               % accel bias
    F(7:9,7:9)   = -skew(w);           % attitude dynamics
    F(7:9,13:15) = -I3;                % gyro bias coupling

    G(4:6,1:3)      = -Rwb;            % accel noise
    G(7:9,4:6)      = -I3;            % gyro noise
    G(10:12,7:9)    = I3;             % accel bias RW
    G(13:15,10:12)  = I3;             % gyro  bias RW

    Phi = eye(15) + F*dt;

    Qa   = (noise.sigma_a)^2   * eye(3);
    Qg   = (noise.sigma_g)^2   * eye(3);
    Qba  = (noise.sigma_ba)^2  * eye(3);
    Qbg  = (noise.sigma_bg)^2  * eye(3);
    Qc   = blkdiag(Qa, Qg, Qba, Qbg);

    Qd = G * Qc * G.' * dt;

    P = Phi * P * Phi.' + Qd;
end

function [state, P] = ekf_update_position(state, P, z, R)
    % Position-only measurement update.
    I3  = eye(3);
    I15 = eye(15);

    H = zeros(3,15);
    H(1:3,1:3) = I3;     % measurement = position

    z_pred = state.p;
    y = z - z_pred;

    S = H * P * H.' + R;
    K = P * H.' / S;

    delta_x = K * y;

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

    P = (I15 - K*H) * P * (I15 - K*H).' + K*R*K.';
end