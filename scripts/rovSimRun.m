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
% Plot the ROV's motions:
plotMotions(t,x);