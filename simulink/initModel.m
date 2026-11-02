function cfg = initModel(cfg)
%INITMODEL Initialise project paths, configuration, and Simulink buses.
% Inputs: cfg, optional defaultConfig output.
% Outputs: cfg, installed in the base workspace for model block parameters.
% Example: cfg = initModel(defaultConfig());

projectRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectRoot));

if nargin < 1 || isempty(cfg)
    if evalin('base', 'exist(''cfg'', ''var'')')
        cfg = evalin('base', 'cfg');
    else
        cfg = defaultConfig();
    end
end
assignin('base', 'cfg', cfg);
defineBuses(cfg);
end
