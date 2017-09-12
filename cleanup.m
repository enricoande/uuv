%% Root directory of this running .m file:
projectRootDir = fileparts(mfilename('fullpath'));

%% Remove project directories from path:
rmpath(fullfile(projectRootDir,'data'));
%rmpath(fullfile(projectRootDir,'documents'));
%rmpath(fullfile(projectRootDir,'libraries'));
rmpath(fullfile(projectRootDir,'models'));
rmpath(fullfile(projectRootDir,'work'));
rmpath(fullfile(projectRootDir,'scripts'));

%% Reset the loction of Simulink-generated files:
Simulink.fileGenControl('reset');

%% leave no trace...
clear projectRootDir;
clear;