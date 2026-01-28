classdef Drone < handle
  properties
    pose
    vel
    quat
    accel
    angularVel

    m
    g
    dt

    thrustCmd
    torqueCmd

    I
    % Geometric controller properties
    kx
    kv
    kR
    kOmega
    desired

    pose_des
    vel_des
    acc_des
    angularVelDes

  end

  % Methods
  methods
    function obj = Drone(params, initStates)
      obj.pose = initStates.Position;
      obj.vel = initStates.Velocity;
      obj.quat = initStates.OrientationQuat;
      obj.angularVel = initStates.AngularVel;
      obj.accel = initStates.Acceleration;

      obj.g = params.g;
      obj.dt = params.dt;
      obj.m = params.m;
      obj.I = [0.232, 0, 0; 0, 0.232, 0; 0, 0, 0.468];

      obj.torqueCmd = [0,0,0];
      obj.thrustCmd = 0;

      obj.pose_des = [30, 0, 90];

      obj.vel_des = [0,0,0]; % Hover state
      obj.acc_des = [0,0,0];

      obj.angularVelDes = [0,0,0];

      % Declare the desired class
      obj.desired.pos = [0,0,90];
      obj.desired.vel = [0,0,0];
      obj.desired.acc = [0,0,0];
      obj.desired.yaw = [0,0,0];
      obj.desired.angularVel = [0,0,0];
      obj.desired.angularAcc = [0,0,0];

      % K values
      obj.kx = 6;
      obj.kv = 4;
      obj.kR = 8;
      obj.kOmega = 2;
    end

    function state = getState(obj)
      state = [obj.pose, obj.vel, obj.accel, obj.quat, obj.angularVel];
    end

    function rotateb2w = rotateBody2World(obj)
      rotateb2w = quat2rotm(obj.quat);
    end

    function obj = updateState(obj, acc, dt)
      obj.dt = dt;

      Rb2w = obj.rotateBody2World();
      thrustWorld = Rb2w * [0; 0; -obj.thrustCmd];
      accWorld = thrustWorld / obj.m + obj.g';

      disp("Acceleration world orientiered"); disp(accWorld);

      obj.vel = obj.vel + accWorld' * obj.dt;
      obj.pose = obj.pose + obj.vel * dt + 0.5 * accWorld' * obj.dt^2;

      % Need to update the angular velocity
      deltaOmega = (inv(obj.I) * obj.torqueCmd) * dt;
      omega_new = obj.angularVel' + deltaOmega;
      obj.angularVel = omega_new';
     
      obj.quat = obj.updateQuaternion(obj.angularVel, obj.dt);
    end

    function q_next = updateQuaternion(obj, omega, dt)
      q = obj.quat;
      Omega = [ 0, -omega(1), -omega(2), -omega(3);
                omega(1),  0,  omega(3), -omega(2);
                omega(2), -omega(3),  0,  omega(1);
                omega(3),  omega(2), -omega(1), 0];
      q_dot = 0.5 * Omega * q';
      q_next = q' + q_dot * dt;
      q_next = q_next' / norm(q_next);
    end

    % Compute errors for the controllers
    function [e_x, e_v, e_R, e_Omega, Psi] = computeErrors(x, v, R, Omega, xd, vd, Rd, Omegad)
      e_x = x - xd;
      e_v = v - vd;

      e_R = 0.5 * obj.vee(Rd' * R - R'*Rd);
      e_Omega = Omega - R' * Rd * Omegad;

      Psi = 0.5 * trace(eye(3) - Rd'*R); % Attitude error scalar, used for stability check, Psi < 2
    end

    function v = vee(~, M)
      v = [M(3, 2); M(1, 3); M(2, 1)];
    end

    function F_des = computeDesiredThrust(~, m, g, x, v, xd, vd, xdd_des, kx, kv)
      e_x = x - xd;
      e_v = v - vd;

      e3 = [0;0;1]; % World axis +z axis, it is gonna be used for rotation fo the b3

      F_des = -kx * e_x -kv * e_v -m * g * e3 + m * xdd_des;
    end

    function Rd = computeDesiredRotation(~, F_des, b1d)
      b3d = F_des / norm(F_des);
      b1d = b1d / norm(b1d);

      % Compute b2d and make it orhogoanalot the other axis again
      b2d = cross(b3d, b1d);
      b2d = b2d / norm(b2d);
      b1d_orth = cross(b2d, b3d);

      Rd = [b1d_orth, b2d, b3d]
    end

    function [f, M] = computeThrustAndMoment(obj, R, Omega, Rd, Omegad, Omegad_dot, F_des, J, kR, kOmega)
      e3 = [0;0;1];
      f = dot(-F_des, R * e3); 
      e_R = 0.5 * obj.vee(Rd' * R - R'*Rd);
      e_Omega = Omega - R' * Rd * Omegad;

      Omega_col = Omega(:);
      gyro_term = cross(Omega_col, J * Omega_col);

      M = -kR * e_R ...
        - kOmega * e_Omega ...
        + gyro_term ...
        - J * (obj.hat(Omega_col) * R' * Rd * Omegad ...
                - R' * Rd * Omegad_dot);
      disp("Torque values"); disp(M);
      disp("Thrust value"); disp(f);
    end

    function Mhat = hat(~, v)
      Mhat = [0 -v(3) v(2);
              v(3) 0 -v(1);
              -v(2) v(1) 0]
    end

    function obj = runController(obj)
      x = obj.pose';
      v = obj.vel'; 

      R = quat2rotm(obj.quat);
      Omega = obj.angularVel(:);

      % Desired States
      xd = obj.desired.pos';
      vd = obj.desired.vel';
      xdd_des = obj.desired.acc';

      yaw_d = obj.desired.yaw;
      yaw_d = double(yaw_d);
      yaw_d = yaw_d(1);

      Omegad = obj.desired.angularVel';
      Omegad_dot = obj.desired.angularAcc';

      % Compute F_des
      F_des = obj.computeDesiredThrust(obj.m, obj.g, x, v, xd, vd, xdd_des, obj.kx, obj.kv);
      b1d = [cos(yaw_d); sin(yaw_d); 0];
      Rd = obj.computeDesiredRotation(F_des, b1d);

      [f, M] = obj.computeThrustAndMoment(R, Omega, Rd, Omegad, Omegad_dot, F_des, obj.I, obj.kR, obj.kOmega);

      obj.thrustCmd = f;
      obj.torqueCmd = M;
    end
  end
end
