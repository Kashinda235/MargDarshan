function tests = testPlanning()
%TESTPLANNING Unit tests for Stage 4 safe-pocket computation.
tests = functiontests(localfunctions);
end

function testOpenRoadProducesLargeConnectedPocket(testCase)
cfg = defaultConfig();
[gridMap, predictions, ego] = clearInputs(cfg);
dynamicRisk = predictionRiskMap(predictions, ego, cfg);
riskField = computeRiskField(gridMap, dynamicRisk, ego, cfg);
pocketState = generateSafePocket(riskField, cfg);
verifyFalse(testCase, pocketState.isEmpty);
verifyFalse(testCase, pocketState.isSmall);
verifyGreaterThan(testCase, pocketState.area, ...
    0.75*cfg.grid.nx*cfg.grid.ny*cfg.grid.resolution^2);
verifyConnected(testCase, pocketState.mask);
verifyTrue(testCase, pocketState.mask( ...
    find(abs(riskField.localY) == min(abs(riskField.localY)),1), ...
    find(abs(riskField.localX) == min(abs(riskField.localX)),1)));
verifyLessThanOrEqual(testCase, pocketState.boundaryCount, ...
    cfg.planner.maxBoundaryVertices);
verifyGreaterThan(testCase, pocketState.quality, 0.5);
end

function testVehicleAheadShrinksForwardDepth(testCase)
cfg = defaultConfig();
[gridMap, predictions, ego] = clearInputs(cfg);
emptyField = computeRiskField(gridMap, ...
    predictionRiskMap(predictions, ego, cfg), ego, cfg);
openPocket = generateSafePocket(emptyField, cfg);

predictions.count = 1;
agent = types.predictedAgent(cfg);
agent.length = cfg.world.agentDimensions(cfg.agentTypes.car, 1);
agent.width = cfg.world.agentDimensions(cfg.agentTypes.car, 2);
agent.confidence = 1;
agent.hypothesisCount = 1;
agent.modeProbability(1) = 1;
agent.hypothesisX(:,1) = 10;
agent.hypothesisY(:,1) = 0;
agent.hypothesisYaw(:,1) = 0;
agent.hypothesisSigma(:,1) = cfg.prediction.initialSigmaM;
predictions.agents(1) = agent;
dynamicRisk = predictionRiskMap(predictions, ego, cfg);
riskField = computeRiskField(gridMap, dynamicRisk, ego, cfg);
blockedPocket = generateSafePocket(riskField, cfg);
verifyLessThan(testCase, blockedPocket.forwardDepth, ...
    openPocket.forwardDepth);
verifyFalse(testCase, blockedPocket.isEmpty);
end

function testBlockedRoadIsSmallAndUnsafeEgoIsEmpty(testCase)
cfg = defaultConfig();
[gridMap, predictions, ego] = clearInputs(cfg);
egoColumn = round((0-gridMap.originX)/cfg.grid.resolution)+1;
egoRow = round((0-gridMap.originY)/cfg.grid.resolution)+1;
blockColumn = egoColumn+round(4.0/cfg.grid.resolution);
gridMap.occupancy(:, blockColumn) = 1;
gridMap.obstacleProb(:, blockColumn) = 1;
gridMap.traversability(:, blockColumn) = 0;
riskField = computeRiskField(gridMap, ...
    predictionRiskMap(predictions, ego, cfg), ego, cfg);
pocketState = generateSafePocket(riskField, cfg);
verifyTrue(testCase, pocketState.isSmall);
verifyLessThan(testCase, pocketState.forwardDepth, 5.0);
verifyConnected(testCase, pocketState.mask);

gridMap.occupancy(egoRow, egoColumn) = 1;
gridMap.obstacleProb(egoRow, egoColumn) = 1;
gridMap.traversability(egoRow, egoColumn) = 0;
riskField = computeRiskField(gridMap, ...
    predictionRiskMap(predictions, ego, cfg), ego, cfg);
emptyPocket = generateSafePocket(riskField, cfg);
verifyTrue(testCase, emptyPocket.isEmpty);
verifyTrue(testCase, emptyPocket.isSmall);
verifyFalse(testCase, any(emptyPocket.mask(:)));
end

function testCorridorPlotAndDemo(testCase)
cfg = defaultConfig();
cfg.pocket.demoDurationS = cfg.pocket.demoDtS;
cfg.pocket.demoPauseS = 0;
[gridMap, predictions, ego] = clearInputs(cfg);
riskField = computeRiskField(gridMap, ...
    predictionRiskMap(predictions, ego, cfg), ego, cfg);
pocketState = generateSafePocket(riskField, cfg);
corridor = pocketCorridor(pocketState, riskField, cfg);
verifyGreaterThan(testCase, corridor.count, 0);
verifyEqual(testCase, corridor.centerline(1,2), 0, ...
    'AbsTol', cfg.grid.resolution);
verifyGreaterThan(testCase, corridor.leftWidth(1), 0);
verifyGreaterThan(testCase, corridor.rightWidth(1), 0);

visibility = get(groot, 'defaultFigureVisible');
visibilityCleanup = onCleanup(@() ...
    set(groot, 'defaultFigureVisible', visibility));
set(groot, 'defaultFigureVisible', 'off');
scenario = createScenario('market', cfg, 1);
figureHandle = figure('Visible', 'off');
figureCleanup = onCleanup(@() closeIfValid(figureHandle));
axesHandle = axes(figureHandle);
verifyEqual(testCase, plotPocket(scenario, scenario.agents, ego, ...
    pocketState, riskField, axesHandle, cfg), axesHandle);
verifyNotEmpty(testCase, findall(axesHandle, 'Type', 'surface'));
demoMetrics = demoPocket(cfg);
verifyEqual(testCase, demoMetrics.framesPerScenario, 1);
if isgraphics(demoMetrics.figure)
    close(demoMetrics.figure);
end
clear figureCleanup visibilityCleanup
end

function testSafePocketStepWarnsWhenOverBudget(testCase)
cfg = defaultConfig();
[gridMap, predictions, ego] = clearInputs(cfg);
dynamicRisk = predictionRiskMap(predictions, ego, cfg);
riskField = computeRiskField(gridMap, dynamicRisk, ego, cfg);
pocketState = generateSafePocket(riskField, cfg);
pocketCorridor(pocketState, riskField, cfg);
startTime = tic;
riskField = computeRiskField(gridMap, dynamicRisk, ego, cfg);
pocketState = generateSafePocket(riskField, cfg);
pocketCorridor(pocketState, riskField, cfg);
elapsedMs = toc(startTime)*1000;
if elapsedMs > cfg.pocket.stepBudgetMs
    warning('testPlanning:StepOverBudget', ...
        'Safe-pocket step took %.2f ms (budget %.2f ms).', ...
        elapsedMs, cfg.pocket.stepBudgetMs);
end
verifyFalse(testCase, pocketState.isEmpty);
end

function testFreeRoadSelectsRouteFollowingTargetSpeed(testCase)
cfg = defaultConfig();
[gridMap, predictions, ego] = clearInputs(cfg);
ego.v = 3.0;
agents = types.agentList(cfg);
reference = straightReference(45.0);
[decision, diagnostics] = planLocal(ego, reference, gridMap, ...
    agents, predictions, [], cfg);
verifyEqual(testCase, numel(diagnostics.candidates), ...
    cfg.planner.numCandidates);
verifyEqual(testCase, decision.candidateCount, ...
    min(cfg.planner.numCandidates, ...
    numel(cfg.planner.lateralOffsetsM)* ...
    numel(cfg.planner.speedFractions)*numel(cfg.planner.offsetTimesS)));
verifySize(testCase, decision.selectedTrajectory.x, ...
    [cfg.planner.samples 1]);
verifyLessThanOrEqual(testCase, ...
    abs(decision.selectedTrajectory.lateralOffset), cfg.grid.resolution);
verifyGreaterThan(testCase, decision.targetSpeed, 0.8* ...
    cfg.adaptive.cruiseTargetSpeedMps);
verifyEqual(testCase, decision.mode, cfg.decisionModes.cruise);
verifySize(testCase, diagnostics.costBreakdown, ...
    [cfg.planner.numCandidates 1]);
end

function testObstacleAheadCausesBrakingOrLateralShift(testCase)
cfg = defaultConfig();
[gridMap, predictions, ego] = clearInputs(cfg);
ego.v = 2.0;
column = round((7.0-gridMap.originX)/cfg.grid.resolution)+1;
centerRow = round((0-gridMap.originY)/cfg.grid.resolution)+1;
rows = centerRow-2:centerRow+2;
gridMap.occupancy(rows,column-1:column+1) = 1;
gridMap.obstacleProb(rows,column-1:column+1) = 1;
gridMap.traversability(rows,column-1:column+1) = 0;
agents = types.agentList(cfg);
[decision, diagnostics] = planLocal(ego, straightReference(45.0), ...
    gridMap, agents, predictions, [], cfg);
shifted = abs(decision.selectedTrajectory.lateralOffset) ...
    > cfg.grid.resolution;
slowed = decision.targetSpeed < ego.v;
verifyTrue(testCase, shifted || slowed || decision.fallbackCode > 0);
verifyGreaterThan(testCase, diagnostics.riskSummary.pocketDepth, 0);
end

function testNoFeasibleCandidateUsesFallback(testCase)
cfg = defaultConfig();
ego = types.egoState();
ego.v = 2.0;
candidates = repmat(types.trajectory(cfg), cfg.planner.numCandidates, 1);
pocketState = types.pocket(cfg);
[selected, selectedIndex, fallbackCode] = selectTrajectory( ...
    candidates, 0, straightReference(20.0), ego, pocketState, [], cfg, 1.0);
verifyEqual(testCase, selectedIndex, 0);
verifyEqual(testCase, fallbackCode, 3);
verifySize(testCase, selected.t, [cfg.planner.samples 1]);
verifyLessThan(testCase, selected.v(end), ego.v);
end

function testAdaptiveModeTransitionsHaveHysteresis(testCase)
cfg = defaultConfig();
summary = struct('agentDensity', 0.012, 'minTTC', 4.5, ...
    'pocketArea', 40.0, 'pocketDepth', 15.0, ...
    'meanPredictionUncertainty', 0.3);
previous = types.decision(cfg);
previous.mode = cfg.decisionModes.cautious;
decision = adaptiveConfig(summary, previous, cfg);
verifyEqual(testCase, decision.mode, cfg.decisionModes.cautious);
summary.agentDensity = 0.005;
summary.minTTC = 6.0;
previous.mode = cfg.decisionModes.cautious;
decision = adaptiveConfig(summary, previous, cfg);
verifyEqual(testCase, decision.mode, cfg.decisionModes.cruise);
summary.minTTC = 2.0;
previous.mode = cfg.decisionModes.emergency;
decision = adaptiveConfig(summary, previous, cfg);
verifyEqual(testCase, decision.mode, cfg.decisionModes.emergency);
end

function testCandidatePlotShowsPaths(testCase)
cfg = defaultConfig();
ego = types.egoState();
decision = struct('targetSpeed', 2.0, 'horizon', 1.0);
[candidates, count] = generateCandidates( ...
    straightReference(20.0), ego, decision, cfg);
figureHandle = figure('Visible', 'off');
cleanup = onCleanup(@() closeIfValid(figureHandle));
axesHandle = axes(figureHandle);
verifyEqual(testCase, plotCandidates(candidates, count, 0, axesHandle), ...
    axesHandle);
verifyGreaterThan(testCase, numel(findall(axesHandle, 'Type', 'line')), 0);
clear cleanup
end

function [gridMap, predictions, ego] = clearInputs(cfg)
gridMap = types.gridMap(cfg);
gridMap.traversability(:) = 1;
predictions = struct('count', 0, ...
    'agents', repmat(types.predictedAgent(cfg), 1, cfg.maxAgents));
ego = types.egoState();
end

function reference = straightReference(lengthM)
reference = struct( ...
    'x', (0:2:lengthM)', ...
    'y', zeros(length((0:2:lengthM)'), 1), ...
    'heading', zeros(length((0:2:lengthM)'), 1), ...
    'curvature', zeros(length((0:2:lengthM)'), 1));
end

function verifyConnected(testCase, mask)
startIndex = find(mask, 1);
if isempty(startIndex)
    verifyFalse(testCase, any(mask(:)));
    return;
end
verifyNotEmpty(testCase, startIndex);
visited = false(size(mask));
queue = zeros(nnz(mask), 1);
queue(1) = startIndex;
head = 1;
tail = 1;
visited(startIndex) = true;
while head <= tail
    [row, column] = ind2sub(size(mask), queue(head));
    head = head+1;
    for rowOffset = -1:1
        for columnOffset = -1:1
            nextRow = row+rowOffset;
            nextColumn = column+columnOffset;
            if nextRow < 1 || nextRow > size(mask,1) || ...
                    nextColumn < 1 || nextColumn > size(mask,2) || ...
                    ~mask(nextRow,nextColumn) || ...
                    visited(nextRow,nextColumn)
                continue;
            end
            tail = tail+1;
            queue(tail) = sub2ind(size(mask), nextRow, nextColumn);
            visited(nextRow,nextColumn) = true;
        end
    end
end
verifyEqual(testCase, visited, mask);
end

function closeIfValid(handle)
if isgraphics(handle)
    close(handle);
end
end
