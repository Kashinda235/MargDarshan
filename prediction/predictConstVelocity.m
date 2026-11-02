function predictions = predictConstVelocity(agentList, cfg)
%PREDICTCONSTVELOCITY Predict agents with constant velocity and growing sigma.
% Inputs: agentList, fixed-capacity world-frame agent list; cfg, defaults.
% Positions/sigma are in m, velocities in m/s, and prediction time in s.
% Outputs: fixed-capacity prediction list with cfg.prediction.steps samples.
% Example: p = predictConstVelocity(scenario.agents, defaultConfig());

if nargin < 2 || isempty(cfg)
    cfg = defaultConfig();
end
validateAgentList(agentList, cfg);
predictions = struct( ...
    'count', 0, ...
    'agents', repmat(types.predictedAgent(cfg), 1, cfg.maxAgents));
steps = cfg.prediction.steps;
times = (0:steps-1)' * cfg.prediction.dt;
for k = 1:agentList.count
    agent = agentList.agents(k);
    if ~agent.valid
        continue;
    end
    model = agentTypeModel(cfg, agent.type);
    velocity = [agent.vx agent.vy];
    speed = hypot(velocity(1), velocity(2));
    if speed > model.maxSpeedMps
        velocity = velocity * (model.maxSpeedMps/speed);
    end
    prediction = types.predictedAgent(cfg);
    prediction.id = agent.id;
    prediction.type = agent.type;
    prediction.length = agent.length;
    prediction.width = agent.width;
    prediction.confidence = agent.confidence;
    prediction.t = times;
    prediction.hypothesisCount = 1;
    prediction.modeProbability(1) = 1;
    prediction.hypothesisX(:, 1) = agent.x + velocity(1)*times;
    prediction.hypothesisY(:, 1) = agent.y + velocity(2)*times;
    if speed > cfg.prediction.forceEpsilonM
        yaw = atan2(velocity(2), velocity(1));
    else
        yaw = agent.yaw;
    end
    prediction.hypothesisYaw(:, 1) = yaw;
    prediction.hypothesisSigma(:, 1) = sqrt( ...
        cfg.prediction.initialSigmaM^2 + ...
        (model.sigmaGrowthMps*times).^2);
    prediction.x = prediction.hypothesisX(:, 1);
    prediction.y = prediction.hypothesisY(:, 1);
    prediction.sigma = prediction.hypothesisSigma(:, 1);
    predictions.count = predictions.count + 1;
    predictions.agents(predictions.count) = prediction;
end
end

function validateAgentList(agentList, cfg)
if ~isstruct(agentList) || ~isfield(agentList, 'count') || ...
        ~isfield(agentList, 'agents') || agentList.count < 0 || ...
        agentList.count > cfg.maxAgents || ...
        numel(agentList.agents) ~= cfg.maxAgents
    error('predictConstVelocity:InvalidAgentList', ...
        'agentList must contain a count and cfg.maxAgents agent records.');
end
end
