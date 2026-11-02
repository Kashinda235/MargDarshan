function predictions = predictAgentsBoid(agentList, ego, roadBoundaries, cfg)
%PREDICTAGENTSBOID Roll out bounded Boid dynamics and lateral hypotheses.
% Inputs: agentList and ego in world-frame ENU; roadBoundaries are polygon
% vertices in m or []; cfg, defaultConfig output. Time is s, yaw rad.
% Outputs: fixed-capacity predictions with weighted paths and sigma in m.
% Example: p = predictAgentsBoid(s.agents, s.egoStart, ...
%     s.road.boundaries(1:s.road.boundaryCount,:), defaultConfig());

if nargin < 4 || isempty(cfg)
    cfg = defaultConfig();
end
if nargin < 3
    roadBoundaries = [];
end
validateAgentList(agentList, cfg);
predictions = struct( ...
    'count', 0, ...
    'agents', repmat(types.predictedAgent(cfg), 1, cfg.maxAgents));
steps = cfg.prediction.steps;
modes = cfg.prediction.modes;
agentIndices = zeros(1, agentList.count);
agentCount = 0;
for k = 1:agentList.count
    if agentList.agents(k).valid
        agentCount = agentCount + 1;
        agentIndices(agentCount) = k;
    end
end
if agentCount == 0
    return;
end
positions = zeros(cfg.maxAgents, modes, 2);
velocities = zeros(cfg.maxAgents, modes, 2);
initialPositions = zeros(cfg.maxAgents, 2);
models = repmat(agentTypeModel(cfg, cfg.agentTypes.car), ...
    1, cfg.maxAgents);
for k = 1:agentCount
    index = agentIndices(k);
    agent = agentList.agents(index);
    model = agentTypeModel(cfg, agent.type);
    models(index) = model;
    initialPositions(index, :) = [agent.x agent.y];
    initialVelocity = [agent.vx agent.vy];
    speed = hypot(initialVelocity(1), initialVelocity(2));
    if speed > model.maxSpeedMps
        initialVelocity = initialVelocity*(model.maxSpeedMps/speed);
    end
    for mode = 1:model.hypothesisCount
        positions(index, mode, :) = [agent.x agent.y];
        velocities(index, mode, :) = initialVelocity;
    end
end
pathX = zeros(cfg.maxAgents, steps, modes);
pathY = zeros(cfg.maxAgents, steps, modes);
pathYaw = zeros(cfg.maxAgents, steps, modes);
pathSigma = zeros(cfg.maxAgents, steps, modes);
probabilities = zeros(cfg.maxAgents, modes);
for k = 1:agentCount
    index = agentIndices(k);
    model = models(index);
    offsets = lateralOffsets(model);
    probabilities(index, 1:model.hypothesisCount) = ...
        hypothesisProbabilities(offsets, model.lateralManoeuvreSpreadM, cfg);
    for mode = 1:model.hypothesisCount
        pathX(index, 1, mode) = agentList.agents(index).x;
        pathY(index, 1, mode) = agentList.agents(index).y;
        pathYaw(index, 1, mode) = agentList.agents(index).yaw;
        pathSigma(index, 1, mode) = cfg.prediction.initialSigmaM;
    end
end

for step = 2:steps
    time = (step-1)*cfg.prediction.dt;
    neighbourList = agentList;
    for k = 1:agentCount
        index = agentIndices(k);
        model = models(index);
        activeModes = model.hypothesisCount;
        neighbourList.agents(index).x = mean( ...
            positions(index, 1:activeModes, 1));
        neighbourList.agents(index).y = mean( ...
            positions(index, 1:activeModes, 2));
        neighbourList.agents(index).vx = mean( ...
            velocities(index, 1:activeModes, 1));
        neighbourList.agents(index).vy = mean( ...
            velocities(index, 1:activeModes, 2));
    end
    predictedEgo = ego;
    predictedEgo.x = ego.x + ego.v*cos(ego.yaw)*time;
    predictedEgo.y = ego.y + ego.v*sin(ego.yaw)*time;
    for k = 1:agentCount
        index = agentIndices(k);
        originalAgent = agentList.agents(index);
        model = models(index);
        offsets = lateralOffsets(model);
        initialYaw = originalAgent.yaw;
        normal = [-sin(initialYaw) cos(initialYaw)];
        for mode = 1:model.hypothesisCount
            currentPosition = reshape(positions(index, mode, :), 1, 2);
            currentVelocity = reshape(velocities(index, mode, :), 1, 2);
            currentAgent = originalAgent;
            currentAgent.x = currentPosition(1);
            currentAgent.y = currentPosition(2);
            currentAgent.vx = currentVelocity(1);
            currentAgent.vy = currentVelocity(2);
            if hypot(currentVelocity(1), currentVelocity(2)) > ...
                    cfg.prediction.forceEpsilonM
                currentAgent.yaw = atan2(currentVelocity(2), ...
                    currentVelocity(1));
            end
            forces = boidForces(currentAgent, neighbourList, ...
                predictedEgo, roadBoundaries, cfg);
            acceleration = forces.separation + forces.alignment + ...
                forces.cohesion + forces.roadBoundary + ...
                forces.goalAttraction;
            currentOffset = dot(currentPosition- ...
                initialPositions(index, :), normal);
            lateralVelocity = dot(currentVelocity, normal);
            offsetProgress = time/cfg.prediction.horizon;
            desiredOffset = offsets(mode)*min(offsetProgress, 1);
            lateralError = desiredOffset-currentOffset;
            acceleration = acceleration + normal*( ...
                cfg.prediction.lateralHypothesisGainPerS2*lateralError - ...
                cfg.prediction.lateralHypothesisDampingPerS* ...
                lateralVelocity);
            randomStd = model.randomAccelerationStdMps2;
            if randomStd > 0
                streamSeed = mod(cfg.prediction.randomSeed + ...
                    1009*abs(originalAgent.id) + 9176*mode + step, ...
                    2^32-1);
                stream = RandStream('mt19937ar', 'Seed', streamSeed);
                acceleration = acceleration + randomStd* ...
                    randn(stream, 1, 2);
            end
            nextVelocity = boundedVelocity(currentVelocity, ...
                acceleration, currentAgent.yaw, model, cfg);
            nextPosition = currentPosition + ...
                nextVelocity*cfg.prediction.dt;
            positions(index, mode, :) = nextPosition;
            velocities(index, mode, :) = nextVelocity;
            pathX(index, step, mode) = nextPosition(1);
            pathY(index, step, mode) = nextPosition(2);
            if hypot(nextVelocity(1), nextVelocity(2)) > ...
                    cfg.prediction.forceEpsilonM
                pathYaw(index, step, mode) = atan2( ...
                    nextVelocity(2), nextVelocity(1));
            else
                pathYaw(index, step, mode) = currentAgent.yaw;
            end
            pathSigma(index, step, mode) = sqrt( ...
                cfg.prediction.initialSigmaM^2 + ...
                (model.sigmaGrowthMps*time)^2);
        end
    end
end

for k = 1:agentCount
    index = agentIndices(k);
    agent = agentList.agents(index);
    model = models(index);
    prediction = types.predictedAgent(cfg);
    prediction.id = agent.id;
    prediction.type = agent.type;
    prediction.length = agent.length;
    prediction.width = agent.width;
    prediction.confidence = agent.confidence;
    prediction.hypothesisCount = model.hypothesisCount;
    prediction.modeProbability = probabilities(index, :);
    prediction.t = (0:steps-1)'*cfg.prediction.dt;
    prediction.hypothesisX(:, 1:model.hypothesisCount) = ...
        reshape(pathX(index, :, 1:model.hypothesisCount), ...
        steps, model.hypothesisCount);
    prediction.hypothesisY(:, 1:model.hypothesisCount) = ...
        reshape(pathY(index, :, 1:model.hypothesisCount), ...
        steps, model.hypothesisCount);
    prediction.hypothesisYaw(:, 1:model.hypothesisCount) = ...
        reshape(pathYaw(index, :, 1:model.hypothesisCount), ...
        steps, model.hypothesisCount);
    prediction.hypothesisSigma(:, 1:model.hypothesisCount) = ...
        reshape(pathSigma(index, :, 1:model.hypothesisCount), ...
        steps, model.hypothesisCount);
    prediction.x = prediction.hypothesisX(:, 1:model.hypothesisCount) * ...
        prediction.modeProbability(1:model.hypothesisCount)';
    prediction.y = prediction.hypothesisY(:, 1:model.hypothesisCount) * ...
        prediction.modeProbability(1:model.hypothesisCount)';
    centeredX = prediction.hypothesisX(:, 1:model.hypothesisCount) - ...
        prediction.x;
    centeredY = prediction.hypothesisY(:, 1:model.hypothesisCount) - ...
        prediction.y;
    baseSigma = prediction.hypothesisSigma(:, 1:model.hypothesisCount);
    prediction.sigma = sqrt(sum( ...
        (centeredX.^2 + centeredY.^2 + baseSigma.^2) .* ...
        prediction.modeProbability(1:model.hypothesisCount), 2));
    predictions.count = predictions.count + 1;
    predictions.agents(predictions.count) = prediction;
end
end

function offsets = lateralOffsets(model)
count = model.hypothesisCount;
if count == 1
    offsets = 0;
else
    offsets = linspace(-model.lateralManoeuvreSpreadM, ...
        model.lateralManoeuvreSpreadM, count);
end
end

function probabilities = hypothesisProbabilities(offsets, spread, cfg)
scale = max(spread, cfg.prediction.forceEpsilonM);
probabilities = exp(-0.5*(offsets/scale).^2);
probabilities = probabilities/sum(probabilities);
end

function nextVelocity = boundedVelocity(velocity, acceleration, ...
        yaw, model, cfg)
forward = [cos(yaw) sin(yaw)];
lateral = [-forward(2) forward(1)];
longitudinalAcceleration = min(max(dot(acceleration, forward), ...
    -model.maxDecelerationMps2), model.maxAccelerationMps2);
lateralAcceleration = min(max(dot(acceleration, lateral), ...
    -model.maxLateralAccelerationMps2), ...
    model.maxLateralAccelerationMps2);
limitedAcceleration = longitudinalAcceleration*forward + ...
    lateralAcceleration*lateral;
nextVelocity = velocity + limitedAcceleration*cfg.prediction.dt;
speed = hypot(nextVelocity(1), nextVelocity(2));
if speed > model.maxSpeedMps
    nextVelocity = nextVelocity*(model.maxSpeedMps/speed);
end
nextSpeed = hypot(nextVelocity(1), nextVelocity(2));
if nextSpeed > cfg.prediction.forceEpsilonM
    targetYaw = atan2(nextVelocity(2), nextVelocity(1));
    yawDifference = atan2(sin(targetYaw-yaw), cos(targetYaw-yaw));
    maximumTurn = model.maxTurnRateRadps*cfg.prediction.dt;
    boundedTurn = min(max(yawDifference, -maximumTurn), maximumTurn);
    boundedYaw = yaw+boundedTurn;
    nextVelocity = nextSpeed*[cos(boundedYaw) sin(boundedYaw)];
end
end

function validateAgentList(agentList, cfg)
if ~isstruct(agentList) || ~isfield(agentList, 'count') || ...
        ~isfield(agentList, 'agents') || agentList.count < 0 || ...
        agentList.count > cfg.maxAgents || ...
        numel(agentList.agents) ~= cfg.maxAgents
    error('predictAgentsBoid:InvalidAgentList', ...
        'agentList must contain a count and cfg.maxAgents agent records.');
end
end
