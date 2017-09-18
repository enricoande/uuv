% rovSimRun.m     e.anderlini@ucl.ac.uk     13/09/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script simulates the dynamics of a ROV.
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
ics = zeros(12,1);
n = [0;0;0;0;0];
rov.weight = 4.92056575e+03;
v_c = [0;0;0;0;0;0];      % current velocity
mdl.tEnd = 20;

tic;
%% Load the Simulink file:
% Simulink file:
sfile = 'rovSim';
% Load the Simulink file:
load_system(sfile);

%% Run the first shot:
sout = sim(sfile,'StopTime',num2str(mdl.tEnd));

%% Close the Simulink file:
close_system(sfile);
toc;

%% Post-processing:
% Extract the data to be plotted:
t = sout.tout;
x = sout.get('logsout').getElement('state').Values.Data;
f = [sout.get('logsout').getElement('thrust').Values.Data,...
    sout.get('logsout').getElement('forces').Values.Data];
% Plot the ROV's motions:
plotMotions(t,x);
% Plot the ROV's forces:
plotForces(t,f);
% % Plot the ROV's path:
% plotPath(t,x);
% % Animate the ROV's motion:
% animateAUV(t,x,50);