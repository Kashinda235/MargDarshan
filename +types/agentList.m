function list = agentList(cfg)
%AGENTLIST Create a fixed-capacity agent list.
% Inputs: cfg, defaultConfig output.
% Outputs: list with count and cfg.maxAgents initialised agent records.
% Example: list = types.agentList(defaultConfig());

if nargin < 1
    cfg = defaultConfig();
end
list = struct( ...
    'count', 0, ...
    'agents', repmat(types.agentState(cfg), 1, cfg.maxAgents));
end
