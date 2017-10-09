% rovSimSetup.m      e.anderlini@ed.ac.uk     13/09/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script initializes the parameters required for the simulation of the
% ROV.
%
% This code has been adapted from the code by Gordon Parker at Michigan
% Technological University.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clear;
% close all;

%% Simulation set-up:
mdl.tStep = 0.01;   % time step length (s)
mdl.tEnd  = 60;     % end time (s)

%% ROV model set-up:
load('../data/rov.mat');  % rov object