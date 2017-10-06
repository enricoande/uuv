% uuvSimRun.m     e.anderlini@ucl.ac.uk     02/10/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script simulates the dynamics of an UUV.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clean up:
clear;
close all;

%% Initialization:
% Run the set-up file:
uuvSimSetup;

% Initial conditions:
ics = zeros(12,1);        % initial conditions (m & rad)
tau = [0;0;0;0;0;0];      % thrusters' thrust (N)
v_c = [0;0;0;0;0;0];      % current velocity (m/s)
% Faheem: you may play with this data and see what happens. Does it match
% your expectations? Note that if you want a circle manoeuver, you will
% need to set a positive value to the thrust vector in heave and yaw.
% You also need to change the UUV data that is read in uuvSimSetup.m and
% you can set the simulation duration in uuvSimSetup.m.

% Correction required for Minerva - remove after changing input data:
rov.weight = 4.92056575e+03;  % correction for incorrect weight (N)

tic;
%% Load the Simulink file:
% Simulink file:
sfile = 'uuvSim_simple';
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
% Faheem: you may comment or uncomment the following commands as you wish.

% Plot the UUV's motions:
plotMotions(t,x);
% Plot the UUV's forces:
plotForces(t,f);
% % Plot the UUV's path:
% plotPath(t,x);
% % Animate the UUV's motion:
% animateAUV(t,x,50,1,8);