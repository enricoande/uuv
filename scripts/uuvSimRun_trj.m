% uuvSimRun_los.m     e.anderlini@ucl.ac.uk     09/10/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script simulates the dynamics of an UUV using trajectory control.
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
rov.M = rov.M_A+rov.M_RB;
v_c = [0;0;0;0;0;0];      % current velocity (m/s)

% Pre-processing:
Tinv = pinv(rov.T);       % inverse of the thrust allocation matrix
n_max = 1200/60;          % max. thrusters' rotational speed (rps)

%% PD controller gains:
kp = [100;100;100;0;0;0];  % proportional gain               
kd = [20;20;20;0;0;0];     % derivative gain

%% Waypoints and trajectory initialization:
waypoints = [0,0,0;
             2,0,0;
             2,4,0;
             2,4,2];

tic;
%% Load the Simulink file:
% Simulink file:
sfile = 'rovSim_trj';
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