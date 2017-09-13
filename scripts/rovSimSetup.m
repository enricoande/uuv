% rovSimSetup.m      e.anderlini@ed.ac.uk     13/09/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script initializes the parameters required for the simulation of the
% point absorber with internal mass and latching control.
%
% N.B.: The model has been generated using the parameters given in:
% Clement and Babarit (2012). 'Discrete control of resonant wave energy 
% devices', Philosophical Transactions of the Royal Society A, 370.
%
% This code has been adapted from the code by Gordon Parker at Michigan
% Technological University.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clear;
% close all;

%% Simulation set-up:
mdl.tStep = 0.01;   % time step length (s)
mdl.tEnd  = 20;     % end time (s)

%% ROV model set-up:
load('../data/rov.mat');  % rob object