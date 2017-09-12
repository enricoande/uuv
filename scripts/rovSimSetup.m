% wecSimSetup.m      E.Anderlini@ed.ac.uk     07/03/2017
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

%% SIMULATION SETUP
mdl.tStep = 0.01;   % time step length (s)
mdl.tEnd  = 20;     % end time (s)
mdl.tStart = 200;   % time for the start of the power averaging (s)

%% WEC MODEL PARAMETERS
% Wave data file:
wave.dataFile = 'waves.mat';

% State-space System Matrices:
ss.dataFile = 'SS.mat';

%% CALCULATION
wave     = updateWaves(wave);
[pto,ss] = updateSS(ss);
 
%% SUPPORT FUNCTIONS
% Update structure for wave excitation:
function waveNew = updateWaves(waveOld)
    waveNew = waveOld;      % make a copy
    load(waveNew.dataFile); % load wave dat
    waveNew.time = time;
    waveNew.elevation = elev;
    waveNew.excitation = excit;
    waveNew.dt = time(2)-time(1);
    
    % Remove the string so that the variable can be passed as a parameter:
    waveNew = rmfield(waveNew,'dataFile');
end

% Update structure for state-space system and other simulation parameters:
function [pto,ssNew] = updateSS(ssOld)
    ssNew = ssOld;        % make a copy
    load(ssNew.dataFile); % load state-space system 

    % extract the multiplier to get the viscous drag force:
    ssNew.drag = drag;

    % Update the PTO variables:
    pto.b2  = b2;
    pto.G   = G;
    pto.eff = eff;
    
    % extract the required matrices - state-space system:
    ssNew.A = A;
    ssNew.B = B;
    
    % Remove the string so that the variable can be passed as a parameter:
    ssNew = rmfield(ssNew,'dataFile');
end