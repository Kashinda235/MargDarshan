function [candidates, costBreakdown] = scoreTrajectories( ...
        candidates, candidateCount, reference, riskField, predictions, ...
        decision, egoState, cfg)
%SCORETRAJECTORIES Compute route, smoothness, speed, collision, and traffic costs.
% Inputs: candidates in world ENU; reference in ego-local m/rad; riskField
% in ego-grid probabilities; predictions in world ENU; decision; egoState;
% cfg. Outputs: updated candidates and fixed cost-breakdown struct arrays.
% Collision risk above cfg.riskThreshold hard-rejects a candidate.
% Example: [paths,costs] = scoreTrajectories(paths,n,ref,field,p,d,ego,cfg);

if nargin < 8 || isempty(cfg)
    cfg = defaultConfig();
end
capacity = cfg.planner.numCandidates;
if numel(candidates) ~= capacity || candidateCount < 0 || ...
        candidateCount > capacity || candidateCount ~= floor(candidateCount)
    error('scoreTrajectories:InvalidCandidates', ...
        'Candidate array and count must match cfg.planner.numCandidates.');
end
expectedRiskSize = [cfg.grid.ny cfg.grid.nx cfg.prediction.steps];
if ~isstruct(riskField) || ~isfield(riskField, 'risk') || ...
        ~isequal(size(riskField.risk), expectedRiskSize)
    error('scoreTrajectories:InvalidRiskField', ...
        'riskField.risk must match the configured grid and prediction steps.');
end
emptyBreakdown = candidates(1).costBreakdown;
costBreakdown = repmat(emptyBreakdown, capacity, 1);
cosYaw = cos(egoState.yaw);
sinYaw = sin(egoState.yaw);
for k = 1:candidateCount
    path = candidates(k);
    localX = cosYaw*(path.x-egoState.x)+sinYaw*(path.y-egoState.y);
    localY = -sinYaw*(path.x-egoState.x)+cosYaw*(path.y-egoState.y);
    routeDistance = zeros(cfg.planner.samples, 1);
    for sample = 1:cfg.planner.samples
        routeDistance(sample) = min(hypot( ...
            localX(sample)-reference.x(:), ...
            localY(sample)-reference.y(:)));
    end
    breakdown = emptyBreakdown;
    breakdown.routeDeviation = mean(routeDistance.^2);
    if cfg.planner.samples > 1
        curvatureChange = diff(path.kappa)./diff(path.t);
        breakdown.curvatureChange = mean(curvatureChange.^2);
        acceleration = gradient(path.v, path.t);
        jerk = gradient(acceleration, path.t);
        breakdown.jerk = mean(jerk.^2);
    end
    breakdown.speedError = mean((path.v-decision.targetSpeed).^2);
    acceleration = gradient(path.v, path.t);
    breakdown.comfort = mean(acceleration.^2);
    sampleRisk = sampleCollisionRisk( ...
        localX, localY, path.t, riskField, cfg);
    breakdown.collisionRisk = mean(sampleRisk);
    breakdown.maxCollisionRisk = max(sampleRisk);
    breakdown.trafficRisk = sampleTrafficRisk(path, predictions, cfg);
    cost = cfg.planner.routeDeviationWeight*breakdown.routeDeviation ...
        + cfg.planner.smoothnessWeight*(breakdown.curvatureChange ...
        + breakdown.jerk) ...
        + cfg.planner.speedWeight*breakdown.speedError ...
        + cfg.planner.comfortWeight*breakdown.comfort ...
        + cfg.planner.collisionRiskWeight*breakdown.collisionRisk ...
        + cfg.planner.trafficWeight*breakdown.trafficRisk;
    candidates(k).cost = cost;
    candidates(k).costBreakdown = breakdown;
    costBreakdown(k) = breakdown;
    maximumRisk = breakdown.maxCollisionRisk;
    if maximumRisk > cfg.riskThreshold && candidates(k).feasible
        candidates(k).feasible = false;
        candidates(k).rejectionCode = 6;
    end
end
end

function risk = sampleCollisionRisk(localX, localY, time, riskField, cfg)
risk = ones(size(time));
for sample = 1:numel(time)
    stepPosition = min(cfg.prediction.steps, max(1, ...
        round(time(sample)/cfg.prediction.dt)+1));
    riskSlice = riskField.risk(:,:,stepPosition);
    risk(sample) = interp2(riskField.localX, riskField.localY, ...
        riskSlice, localX(sample), localY(sample), 'linear', 1.0);
end
risk = min(1.0, max(0.0, risk));
end

function totalRisk = sampleTrafficRisk(path, predictions, cfg)
if ~isstruct(predictions) || ~all(isfield(predictions, {'count','agents'})) || ...
        predictions.count < 0 || predictions.count > cfg.maxAgents || ...
        numel(predictions.agents) ~= cfg.maxAgents
    error('scoreTrajectories:InvalidPredictions', ...
        'predictions must contain a valid fixed-capacity agent list.');
end
combinedRisk = zeros(cfg.planner.samples, 1);
for agentIndex = 1:predictions.count
    agent = predictions.agents(agentIndex);
    if agent.type < 1 || agent.type > numel(cfg.planner.trafficTypeWeights)
        error('scoreTrajectories:InvalidAgentType', ...
            'Predicted agent type is outside the configured type range.');
    end
    for mode = 1:agent.hypothesisCount
        probability = agent.confidence*agent.modeProbability(mode);
        if probability <= 0
            continue;
        end
        agentX = interp1(agent.t, agent.hypothesisX(:,mode), ...
            path.t, 'linear', 'extrap');
        agentY = interp1(agent.t, agent.hypothesisY(:,mode), ...
            path.t, 'linear', 'extrap');
        sigma = interp1(agent.t, agent.hypothesisSigma(:,mode), ...
            path.t, 'linear', 'extrap');
        separation = hypot(path.x-agentX, path.y-agentY);
        scale = cfg.planner.trafficRiskScaleM ...
            +(cfg.vehicle.width+agent.width)/2+max(0,sigma);
        modeRisk = min(1.0, max(0.0, probability ...
            *cfg.planner.trafficTypeWeights(agent.type) ...
            .*exp(-0.5*(separation./scale).^2)));
        combinedRisk = 1-(1-combinedRisk).*(1-modeRisk);
    end
end
totalRisk = mean(combinedRisk);
end
