%% Prepare the environment
clc
clear all
close all

addpath('simulator');

%% Define the simulation

% Define the hardware architecture
m = robots.vtol_custom();
% Define the world
average_wind = [];
w = worlds.empty_world(average_wind, false);

% Define the controller
c = controllers.vtol_controllers(m);

% Define the simulation object
sim = simulation(m, c, w);

%% Initial multirotor state

pos = [0; 0; -200];

% vel = [17.3005; 0; 0];

vel = [0; 0; 0];

rpy = [0; 0; 0];
omega = [0; 0; 0];
lastThrust = [0;0;0];
sim.Multirotor.SetInitialState(pos, vel, rpy, omega,lastThrust);

%% Get the controller response(s)

% Simulate trajectory following
% [traj, total_time] = trajectories.vtol_simple();
% sim.SetTotalTime(total_time);
% pos_thresh = 20;
% rpy_thresh = 5; 
% force_thresh = 0;
% sim.SimulateTrajectory(traj, pos_thresh, rpy_thresh, force_thresh);

% Or simulate attitude response
% sim.SetTotalTime(10);
% figure; 
% sim.SimulateAttitudeResponse([10; 0; 0], true);

% Or simulate position response
sim.SetTotalTime(20);
figure;
sim.SimulatePositionResponse([600; 0; -200], 0, true);

%% Draw Additional plots
% graphics.PlotSignalsByName(2, {'laileronp', 'laileronr', 'raileronp', 'raileronr'});
graphics.PlotSignalsByName(3, {'pos', 'vel', 'accel', 'rpy', 'euler deriv', 'ang accel', 'sat'}, false, true);
graphics.PlotSignalsByName(2, {'servo', 'inward', 'sideward'}, false, true);
%graphics.PlotSignalsByName(1, {'accel'}, false, true);
% graphics.PlotSignalsByName(2, {'alpha', 'beta', 'airspeed'});
% graphics.PlotSignalsByName(2, {'omega','airspeed'});

%% Animate the result


fpv_cam = camera;
fpv_cam.Offset = [0; 0; -0.35];
graphics.AnimateLoggedTrajectory(sim.Multirotor, sim.Environment, 9, 1, true, true, []);
% graphics.RecordLoggedTrajectoryAnimation('myvideo', 30, sim.Multirotor, sim.Environment, 0, 1, true, true, fpv_cam);
