clear all; close all; clc;

addpath("../lib/");

scene = uavScenario(ReferenceLocation=[40.707088 -74.012146 0], UpdateRate=10);

% Mesh information for Manhattan buildings and the terrain model
% addMesh(scene, "terrain", {"gmted2010", [-200 200], [-200 200]}, [0.6 0.6 0.6]);
% addMesh(scene, "buildings", {"manhattan.osm", [-150 150], [-150 150], "auto"}, [0.6431 0.8706 0.6275]);

plat = uavPlatform("UAV", scene, "ReferenceFrame", "ENU"); % The used coordinate system is ENU (East-North-Up)
updateMesh(plat, "quadrotor", {2}, [1 0 0], eye(4));
setup(scene);
ax = show3D(scene);

view(3); grid on;

dt = 1 / scene.UpdateRate;
totalTime = 10; % Can be tuned
gravity = [0, 0, -9.81]; % In the ENU system

initStates.Position = [0, 0, 100]; % As the starting Position
initStates.Velocity = [0, 40, 0];
initStates.OrientationQuat = [1, 0, 0, 0];
initStates.AngularVel = [0, 0, deg2rad(30)];
initStates.Acceleration = gravity;

% Keep track of the simulation path
trajectory = [];
gpsTrajectory = [];
thrustHist = [];
timeHist = [];

% Drone parameters
groundTruthParams.g = gravity;
groundTruthParams.dt = dt;
groundTruthParams.m = 1.25; % Weight of the drone

drone1 = Drone(groundTruthParams, initStates);

% Simulation loop
for t =  0:dt:totalTime
  % Drone crash check
  if drone1.pose(3) < -3
    % msgbox('Crashed!', 'Error', 'error');
    break;
  end

  trajectory = [trajectory; drone1.pose];
  disp("Drone acceleration"); disp(drone1.accel);

  drone1.runController();

  thrustHist = [thrustHist; drone1.thrustCmd];
  timeHist = [timeHist; t];

  drone1State = drone1.getState();
  
  % Print out all of the state values (Only z axis)
  fprintf("Drone position %d \n", drone1.pose(3));  
  drone1.updateState(drone1.accel, dt);
  move(plat, drone1State);

  advance(scene);
  show3D(scene, "parent", ax);

  hold on;
  plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), 'b-', 'LineWidth', 2); % Might be wrong parameters
  hold off;
  drawnow;
  pause(0.01);

end

figure;
plot(timeHist, thrustHist, 'r-', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Thrust Command [N]');
title('Thrust Command Over Time');
grid on;
