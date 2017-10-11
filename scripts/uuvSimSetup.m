% uuvSimSetup.m      e.anderlini@ed.ac.uk     02/10/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script initializes the parameters required for the simulation of an
% unmanned underwater vehicle (UUV).
%
% This code has been adapted from the code by Gordon Parker at Michigan
% Technological University.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clear;
% close all;

%% Simulation set-up:
mdl.tStep = 0.01;         % time step length (s)
mdl.tEnd  = 20;           % end time (s)

%% UUV model set-up:
load('rov.mat');  % rov object
% Faheem: you will beed to change this with the REMUS data. The object is 
% generated with the function readData.m in the directory ../preprocessing.
% You will need to create a new folder similar to ../preprocessing/Minerva 
% with the data saved in the same format. Otherwise, you can create a new 
% function to save the data, but please save the data to the same object.