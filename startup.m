%% Root directory of this running .m file:
projectRootDir = fileparts(mfilename('fullpath'));

%% Add project directories to path:
addpath(fullfile(projectRootDir,'data'),'-end');
%addpath(fullfile(projectRootDir,'functions'),'-end');
addpath(fullfile(projectRootDir,'models'),'-end');
addpath(fullfile(projectRootDir,'work'),'-end');
addpath(fullfile(projectRootDir,'scripts'),'-end');

%% Save Simulink-generated helper files to work
Simulink.fileGenControl('set',...
    'CacheFolder',fullfile(projectRootDir,'work'),...
    'CodeGenFolder',fullfile(projectRootDir,'work'));

% load bus object data types
%load battery_bus_types
%load boost_converter_bus_types
%load bus_load_bus_types
%load source_bus
%load guidance_bus
%load grid_state_bus