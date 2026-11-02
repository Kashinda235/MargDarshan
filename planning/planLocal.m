function [decision, diagnostics] = planLocal(egoState, reference, gridMap, ...
        agentList, predictions, previousDecision, cfg)
%PLANLOCAL Run adaptive candidate generation, checking, scoring, and selection.
% Inputs: egoState in world ENU (m/rad, m/s); reference in ego-local
% x/y (m), heading (rad), curvature (1/m); gridMap; fixed-capacity
% agentList/predictions; previous decision (optional); cfg.
% Outputs: decision with selected world-frame trajectory and mode policy;
% diagnostics with fixed candidate, feasibility, cost, risk, and pocket data.
% Example: [d,debug] = planLocal(ego,ref,map,agents,predictions,[],cfg);

if nargin < 7 || isempty(cfg)
    cfg = defaultConfig();
end
if nargin < 6
    previousDecision = [];
end
if ~isstruct(agentList) || ~all(isfield(agentList, {'count','agents'})) || ...
        agentList.count < 0 || agentList.count > cfg.maxAgents || ...
        numel(agentList.agents) ~= cfg.maxAgents
    error('planLocal:InvalidAgentList', ...
        'agentList must have cfg.maxAgents records and a valid count.');
end
dynamicRisk = predictionRiskMap(predictions, egoState, cfg);
riskField = computeRiskField(gridMap, dynamicRisk, egoState, cfg);
pocketState = generateSafePocket(riskField, cfg);
riskSummary = summarizeRisk(egoState, agentList, predictions, ...
    pocketState, cfg);
adaptiveDecision = adaptiveConfig(riskSummary, previousDecision, cfg);
modeCfg = cfg;
modeCfg.pocket.safetyMarginM = adaptiveDecision.safetyMargin;
riskField = computeRiskField(gridMap, dynamicRisk, egoState, modeCfg);
pocketState = generateSafePocket(riskField, modeCfg);
[candidates, candidateCount] = generateCandidates( ...
    reference, egoState, adaptiveDecision, cfg);
[candidates, feasibility] = checkFeasibility( ...
    candidates, candidateCount, pocketState, egoState, cfg);
[candidates, costBreakdown] = scoreTrajectories(candidates, ...
    candidateCount, reference, riskField, predictions, ...
    adaptiveDecision, egoState, cfg);
[selected, selectedIndex, fallbackCode, candidates] = ...
    selectTrajectory(candidates, candidateCount, reference, egoState, ...
    pocketState, previousDecision, cfg, adaptiveDecision.horizon);
for k = 1:candidateCount
    costBreakdown(k) = candidates(k).costBreakdown;
end

decision = types.decision(cfg);
decision.selectedTrajectory = selected;
decision.targetSpeed = selected.targetSpeed;
decision.safetyMargin = adaptiveDecision.safetyMargin;
decision.horizon = adaptiveDecision.horizon;
decision.replanPeriod = adaptiveDecision.replanPeriod;
decision.mode = adaptiveDecision.mode;
decision.candidateCount = candidateCount;
decision.selectedIndex = selectedIndex;
decision.fallbackCode = fallbackCode;
diagnostics = struct( ...
    'candidates', candidates, ...
    'candidateCount', candidateCount, ...
    'feasibility', feasibility, ...
    'costBreakdown', costBreakdown, ...
    'riskSummary', riskSummary, ...
    'riskField', riskField, ...
    'pocketState', pocketState);
end

function summary = summarizeRisk(egoState, agentList, predictions, ...
        pocketState, cfg)
cosYaw = cos(egoState.yaw);
sinYaw = sin(egoState.yaw);
egoVelocity = egoState.v*[cosYaw sinYaw];
agentCountInRadius = 0;
minimumTTC = inf;
for k = 1:agentList.count
    agent = agentList.agents(k);
    if ~agent.valid
        continue;
    end
    displacement = [agent.x-egoState.x, agent.y-egoState.y];
    localX = cosYaw*displacement(1)+sinYaw*displacement(2);
    localY = -sinYaw*displacement(1)+cosYaw*displacement(2);
    if hypot(localX, localY) <= cfg.adaptive.agentRadiusM
        agentCountInRadius = agentCountInRadius+1;
    end
    relativeVelocity = [agent.vx agent.vy]-egoVelocity;
    distance = max(0, hypot(displacement(1),displacement(2)) ...
        -(cfg.vehicle.length+agent.length)/2);
    closingSpeed = -dot(displacement, relativeVelocity) ...
        /max(hypot(displacement(1),displacement(2)), eps);
    if closingSpeed > 0
        minimumTTC = min(minimumTTC, distance/closingSpeed);
    end
end
meanUncertainty = 0.0;
uncertaintyCount = 0;
if ~isstruct(predictions) || ~all(isfield(predictions, {'count','agents'})) ...
        || predictions.count < 0 || predictions.count > cfg.maxAgents || ...
        numel(predictions.agents) ~= cfg.maxAgents
    error('planLocal:InvalidPredictions', ...
        'predictions must have cfg.maxAgents records and a valid count.');
end
for k = 1:predictions.count
    prediction = predictions.agents(k);
    if prediction.hypothesisCount > 0
        modeWeights = prediction.modeProbability( ...
            1:prediction.hypothesisCount);
        weights = repmat(modeWeights, cfg.prediction.steps, 1);
        values = prediction.hypothesisSigma(:,1:prediction.hypothesisCount);
        valid = isfinite(values) & values >= 0 & weights > 0;
        meanUncertainty = meanUncertainty+sum(values(valid).*weights(valid));
        uncertaintyCount = uncertaintyCount+sum(weights(valid));
    else
        finiteValues = prediction.sigma(isfinite(prediction.sigma) ...
            & prediction.sigma >= 0);
        meanUncertainty = meanUncertainty+sum(finiteValues);
        uncertaintyCount = uncertaintyCount+numel(finiteValues);
    end
end
if uncertaintyCount > 0
    meanUncertainty = meanUncertainty/uncertaintyCount;
end
summary = struct( ...
    'agentDensity', agentCountInRadius/ ...
        (pi*cfg.adaptive.agentRadiusM^2), ...
    'minTTC', minimumTTC, ...
    'pocketArea', pocketState.area, ...
    'pocketDepth', pocketState.forwardDepth, ...
    'meanPredictionUncertainty', meanUncertainty);
end
