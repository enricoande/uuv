%% Root directory of this running .m file:
projectRootDir = fileparts(mfilename('fullpath'));

%% Add project directories to path:
addpath(fullfile(projectRootDir,'data'),'-end');
addpath(fullfile(projectRootDir,'functions'),'-end');
addpath(fullfile(projectRootDir,'models'),'-end');
addpath(fullfile(projectRootDir,'work'),'-end');
addpath(fullfile(projectRootDir,'scripts'),'-end');

%% Save Simulink-generated helper files to work
Simulink.fileGenControl('set',...
    'CacheFolder',fullfile(projectRootDir,'work'),...
    'CodeGenFolder',fullfile(projectRootDir,'work'));