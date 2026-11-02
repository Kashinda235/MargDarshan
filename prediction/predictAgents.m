function predictions = predictAgents(agentList, ego, roadBoundaries, cfg)
%PREDICTAGENTS Select the configured agent predictor behind one interface.
% Inputs: agentList, world-frame agent list; ego, world-frame ego state;
% roadBoundaries, world-frame polygon vertices in m or []; cfg, defaults.
% Outputs: fixed-capacity predictions with cfg.prediction.steps samples.
% Example: p = predictAgents(s.agents, s.egoStart, [], defaultConfig());

if nargin < 4 || isempty(cfg)
    cfg = defaultConfig();
end
if nargin < 3
    roadBoundaries = [];
end
switch lower(char(cfg.prediction.method))
    case 'cv'
        predictions = predictConstVelocity(agentList, cfg);
    case 'boid'
        predictions = predictAgentsBoid( ...
            agentList, ego, roadBoundaries, cfg);
    case 'nn'
        error('predictAgents:UnavailableMethod', ...
            'The nn predictor is not implemented in this stage.');
    otherwise
        error('predictAgents:UnknownMethod', ...
            'cfg.prediction.method must be cv, boid, or nn.');
end
end
