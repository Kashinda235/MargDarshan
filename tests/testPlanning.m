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

function [gridMap, predictions, ego] = clearInputs(cfg)
gridMap = types.gridMap(cfg);
gridMap.traversability(:) = 1;
predictions = struct('count', 0, ...
    'agents', repmat(types.predictedAgent(cfg), 1, cfg.maxAgents));
ego = types.egoState();
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
