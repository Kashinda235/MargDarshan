function metrics = demoPerception(cfg)
%DEMOPERCEPTION Run the 20 s noisy-sensor market perception demonstration.
% Inputs: cfg, optional defaultConfig output; ego follows a scripted
% straight path at the configured speed. Time in s, positions in m.
% Outputs: metrics with mean position error (m), ID-switch count, and a
% figure comparing current tracked agents with ground truth.
% Example: metrics = demoPerception();

if nargin < 1 || isempty(cfg)
    cfg = defaultConfig();
end
scenario = createScenario('market', cfg, cfg.perception.randomSeed);
truth = scenario.agents;
ego = scenario.egoStart;
dt = cfg.simulation.dt;
frameCount = round(cfg.perception.demoDurationS/dt);
tracker = types.trackerState(cfg);
mapState = types.gridMapState(cfg);
randomState = cfg.perception.randomSeed;
previousTrackId = zeros(1, cfg.maxAgents);
errorSum = 0.0;
errorCount = 0;
switchCount = 0;

for frameIndex = 1:frameCount
    ego.v = cfg.perception.demoEgoSpeedMps;
    [truth, ~, scenario] = stepWorld( ...
        scenario, truth, ego, dt, cfg);
    ego.x = ego.x+ego.v*cos(ego.yaw)*dt;
    ego.y = ego.y+ego.v*sin(ego.yaw)*dt;
    [sensorFrame, randomState] = simulateSensors( ...
        scenario, truth, ego, cfg, randomState);
    fused = fuseDetections(sensorFrame, cfg);
    [tracked, tracker] = trackAgents(fused, tracker, dt, cfg);
    [~, mapState] = buildGridMap( ...
        scenario, fused, ego, mapState, dt, cfg);
    [pairs, pairCount] = matchForEvaluation( ...
        truth, tracked, cfg.perception.evaluationAssociationGateM);
    for pairIndex = 1:pairCount
        truthAgent = truth.agents(pairs(pairIndex,1));
        trackedAgent = tracked.agents(pairs(pairIndex,2));
        errorSum = errorSum+hypot( ...
            truthAgent.x-trackedAgent.x, truthAgent.y-trackedAgent.y);
        errorCount = errorCount+1;
        if truthAgent.id >= 1 && truthAgent.id <= cfg.maxAgents
            previous = previousTrackId(truthAgent.id);
            if previous > 0 && previous ~= trackedAgent.id
                switchCount = switchCount+1;
            end
            previousTrackId(truthAgent.id) = trackedAgent.id;
        end
    end
end

if errorCount > 0
    meanError = errorSum/errorCount;
else
    meanError = NaN;
end
figureHandle = figure('Name', 'MargDarshan Stage 2 perception', ...
    'Color', 'w');
axesHandle = axes(figureHandle);
plotWorld(scenario, truth, ego, axesHandle, cfg);
hold(axesHandle, 'on');
truthX = zeros(1, truth.count);
truthY = zeros(1, truth.count);
for k = 1:truth.count
    truthX(k) = truth.agents(k).x;
    truthY(k) = truth.agents(k).y;
end
trackX = zeros(1, tracked.count);
trackY = zeros(1, tracked.count);
for k = 1:tracked.count
    trackX(k) = tracked.agents(k).x;
    trackY(k) = tracked.agents(k).y;
end
scatter(axesHandle, truthX, truthY, 48, 'o', ...
    'MarkerEdgeColor', [0.0 0.35 0.8], 'LineWidth', 1.5);
scatter(axesHandle, trackX, trackY, 55, 'x', ...
    'MarkerEdgeColor', [0.85 0.1 0.1], 'LineWidth', 1.8);
[finalPairs, finalPairCount] = matchForEvaluation( ...
    truth, tracked, cfg.perception.evaluationAssociationGateM);
for k = 1:finalPairCount
    truthAgent = truth.agents(finalPairs(k,1));
    trackedAgent = tracked.agents(finalPairs(k,2));
    plot(axesHandle, [truthAgent.x trackedAgent.x], ...
        [truthAgent.y trackedAgent.y], '-', ...
        'Color', [0.2 0.2 0.2], 'LineWidth', 0.8);
end
legend(axesHandle, {'Ground truth', 'Tracked agents'}, ...
    'Location', 'best');
title(axesHandle, sprintf( ...
    'Market perception at %.1f s | mean error %.2f m | ID switches %d', ...
    scenario.time, meanError, switchCount));
hold(axesHandle, 'off');

metrics = struct('meanPositionErrorM', meanError, ...
    'idSwitchCount', switchCount, 'figure', figureHandle);
fprintf('Mean tracked-agent position error: %.3f m\n', meanError);
fprintf('ID-switch count: %d\n', switchCount);
end

function [pairs, pairCount] = matchForEvaluation(truth, tracked, gate)
capacity = truth.count*tracked.count;
candidates = zeros(capacity, 3);
candidateCount = 0;
for truthIndex = 1:truth.count
    for trackIndex = 1:tracked.count
        distance = hypot(truth.agents(truthIndex).x ...
            - tracked.agents(trackIndex).x, ...
            truth.agents(truthIndex).y ...
            - tracked.agents(trackIndex).y);
        if distance <= gate
            candidateCount = candidateCount+1;
            candidates(candidateCount,:) = ...
                [distance truthIndex trackIndex];
        end
    end
end
candidates = sortrows(candidates(1:candidateCount,:), 1);
pairs = zeros(min(truth.count, tracked.count), 2);
truthUsed = false(1, truth.count);
trackUsed = false(1, tracked.count);
pairCount = 0;
for k = 1:candidateCount
    truthIndex = candidates(k,2);
    trackIndex = candidates(k,3);
    if truthUsed(truthIndex) || trackUsed(trackIndex)
        continue
    end
    pairCount = pairCount+1;
    pairs(pairCount,:) = [truthIndex trackIndex];
    truthUsed(truthIndex) = true;
    trackUsed(trackIndex) = true;
end
end
