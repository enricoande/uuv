% uuvSimRun_los.m     e.anderlini@ucl.ac.uk     10/10/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script simulates the dynamics of an UUV using LOS guidance.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clean up:
clear;
close all;

%% Suppress warning messages:
%warning('off', 'Simulink:SimState:SimStateParameterChecksumMisMatch');

%% Initialization:
% Run the set-up file:
rovSimSetup;

% Initial conditions:
ics = zeros(12,1);        % initial conditions (m & rad)
% n = [0;0;0;700;1000];     % thrusters' rpm
% n = n/60;                 % thrusters' rps
%tau = [0;0;0;0;0;0];     % thrusters' thrust (N)
% rov.weight = 4.92056575e+03;  % correction for incorrect weight (N)
rov.weight = rov.buoyancy;
v_c = [0;0;0;0;0;0];      % current velocity (m/s)

% Pre-processing:
Tinv = pinv(rov.T);       % inverse of the thrust allocation matrix
n_max = 1200/60;          % max. thrusters' rotational speed (rps)

%% PID controller gains:
% You will need to obtain the gains by trial and error. Look at the
% following sections one by one to prevent problems.
% Depth: - PID controller
% zd = 0;                    % desired depth (m)
kpd = 120;                 % proportional gain               
kid = 15;                  % integral gain
kdd = 30;                  % derivative gain
% Speed: - PID controller
ud = 0.1;                  % desired speed (m/s)
kpu = 100;                 % proportional gain
kdu = 10;                  % integral gain
kiu = 20;                  % derivative gain
% Heading: - PID controller
% psid = 0;                  % desired heading (rad)
kppsi = 150;               % proportional gain
kdpsi = 20;                % integral gain
kipsi = 30;                 
% Steering: PID controller
kps = 100;                 % proportional gain
kis = 10;                  % integral gain
kds = 20;                  % derivative gain

%% Waypoints and circle of acceptance:
waypoints = [0,0,0;
             2,0,0;
             2,4,0];
r = 0.1;

tic;
%% Load the Simulink file:
% Simulink file:
sfile = 'uuvSim_los';
% Load the Simulink file:
load_system(sfile);

%% Run the first shot:
sout = sim(sfile,'StopTime',num2str(mdl.tEnd));

%% Close the Simulink file:
% close_system(sfile);
toc;

%% Post-processing:
% Extract the data to be plotted:
t = sout.tout;
x = sout.get('logsout').getElement('state').Values.Data;
f = [sout.get('logsout').getElement('thrust').Values.Data,...
    sout.get('logsout').getElement('forces').Values.Data];
% Plot the ROV's motions:
plotMotions(t,x);
% % Plot the ROV's forces:
% plotForces(t,f);
% Plot the ROV's path:
plotPath(x,waypoints);
% % Animate the ROV's motion:
% animateAUV(t,x,50,1,8);