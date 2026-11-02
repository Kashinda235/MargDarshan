function startup()
%STARTUP Add the MargDarshan project folders to the MATLAB path.
% Inputs: none.
% Outputs: none; adds project folders to the MATLAB path.
% Example: startup();

projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(projectRoot));
end
