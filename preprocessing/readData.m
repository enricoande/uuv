% readData.m     e.anderlini@ucl.ac.uk     13/09/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script is used to extract the ROV data in the provided folder and
% create a Matlab object saved in a Matlab format.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;

%% Input data:
dir_name = './Minerva/';      % directory name
rov.g = 9.807;                % gravitational constant (m/s2)
rov.density = 1025;           % sea water density (kg/m3)

%% Read the input data:
rov.B         = importdata([dir_name,'COB.txt']);
rov.G         = importdata([dir_name,'COG.txt']);
rov.prop_diam = importdata([dir_name,'D.txt']);
rov.D_L       = importdata([dir_name,'D_L.txt']);
rov.D_q       = importdata([dir_name,'D_q.txt']);
rov.K_T       = importdata([dir_name,'K_T.txt']);
rov.M_A       = importdata([dir_name,'M_A.txt']);
rov.M_RB      = importdata([dir_name,'M_RB.txt']);
rov.T         = importdata([dir_name,'T.txt']);
rov.theta     = importdata([dir_name,'theta.txt']);
rov.volume    = importdata([dir_name,'V.txt']);

%% Compute the remaining constants:
rov.mass     = rov.M_RB(1,1);
rov.inertia  = rov.M_RB(4:end,4:end);
rov.buoyancy = rov.volume*rov.density*rov.g;
rov.weight   = rov.mass*rov.g;
rov.O        = zeros(3);
rov.inv_mass = pinv(rov.M_RB+rov.M_A);
rov.S_r_g    = skew(rov.G);

%% Store the generated data in a Matlab file:
save('../data/rov.mat','rov');