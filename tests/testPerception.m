function tests = testPerception()
%TESTPERCEPTION Unit tests for simulated Stage 2 perception.
tests = functiontests(localfunctions);
end

function testExplicitRandomGenerators(testCase)
[first, state] = perceptionRandomNormal(23);
[second, stateAgain] = perceptionRandomNormal(23);
verifyEqual(testCase, first, second);
verifyEqual(testCase, state, stateAgain);
[uniform, ~] = perceptionRandomUniform(23);
verifyGreaterThan(testCase, uniform, 0);
verifyLessThan(testCase, uniform, 1);
end

function testSensorSimulationIsReproducibleAndBounded(testCase)
cfg = defaultConfig();
cfg.perception.cameraDetectionBase = 1.0;
cfg.perception.cameraDetectionDecayM = 1000000.0;
cfg.perception.lidarDetectionProbability = 1.0;
cfg.perception.radarDetectionProbability = 1.0;
cfg.perception.cameraClassConfusionProb = 0.0;
cfg.perception.cameraFalsePositiveMean = 0.0;
cfg.perception.lidarFalsePositiveMean = 0.0;
cfg.perception.radarFalsePositiveMean = 0.0;
scenario = createScenario('market', cfg, 7);
[first, firstState] = simulateSensors( ...
    scenario, scenario.agents, scenario.egoStart, cfg, 31);
[second, secondState] = simulateSensors( ...
    scenario, scenario.agents, scenario.egoStart, cfg, 31);
verifyEqual(testCase, first, second);
verifyEqual(testCase, firstState, secondState);
verifyGreaterThan(testCase, first.camera.count, 0);
verifyGreaterThan(testCase, first.lidar.count, 0);
verifyGreaterThan(testCase, first.radar.count, 0);
verifyLessThanOrEqual(testCase, first.camera.count, ...
    cfg.perception.maxDetections);
verifySize(testCase, first.camera.items(1).bbox, [1 4]);
end

function testCameraConfusionAndLidarOcclusion(testCase)
cfg = defaultConfig();
cfg.perception.cameraDetectionBase = 1.0;
cfg.perception.cameraDetectionDecayM = 1000000.0;
cfg.perception.cameraClassConfusionProb = 1.0;
cfg.perception.lidarDetectionProbability = 1.0;
cfg.perception.radarDetectionProbability = 1.0;
cfg.perception.cameraFalsePositiveMean = 0.0;
cfg.perception.lidarFalsePositiveMean = 0.0;
cfg.perception.radarFalsePositiveMean = 0.0;
scenario = createScenario('market', cfg, 7);
scenario.obstacles.count = 0;
truth = types.agentList(cfg);
truth.count = 2;
truth.agents(1) = makeTruthAgent(cfg, 1, 10.0);
truth.agents(2) = makeTruthAgent(cfg, 2, 20.0);
ego = types.egoState();
frame = simulateSensors(scenario, truth, ego, cfg, 19);
verifyEqual(testCase, frame.camera.count, 2);
verifyEqual(testCase, frame.lidar.count, 1);
verifyEqual(testCase, frame.radar.count, 2);
for k = 1:frame.camera.count
    verifyNotEqual(testCase, frame.camera.items(k).type, cfg.agentTypes.car);
end
verifyGreaterThan(testCase, cfg.perception.radarPositionNoiseStdM, ...
    cfg.perception.lidarPositionNoiseStdM);
end

function testFusionAssociatesAvailableSensors(testCase)
cfg = deterministicSensorConfig();
scenario = createScenario('market', cfg, 7);
frame = simulateSensors(scenario, scenario.agents, ...
    scenario.egoStart, cfg, 41);
fused = fuseDetections(frame, cfg);
verifyGreaterThan(testCase, fused.count, 0);
for k = 1:fused.count
    detection = fused.items(k);
    verifyTrue(testCase, detection.hasCamera ...
        || detection.hasLidar || detection.hasRadar);
    verifyGreaterThanOrEqual(testCase, detection.confidence, 0);
    verifyLessThanOrEqual(testCase, detection.confidence, 1);
    verifyGreaterThan(testCase, detection.length, 0);
    verifyGreaterThan(testCase, detection.width, 0);
end
end

function testTrackerKeepsIdentityAcrossNoisyStraightPass(testCase)
cfg = defaultConfig();
state = types.trackerState(cfg);
trackId = 0;
for k = 1:8
    fused = oneFusedDetection(0.8*k+0.03*(-1)^k, ...
        0.04*(-1)^k, cfg);
    [tracked, state] = trackAgents(fused, state, 0.1, cfg);
    if k >= cfg.perception.trackerConfirmationHits
        verifyEqual(testCase, tracked.count, 1);
        if trackId == 0
            trackId = tracked.agents(1).id;
        end
        verifyEqual(testCase, tracked.agents(1).id, trackId);
        verifyLessThan(testCase, abs( ...
            tracked.agents(1).x-0.8*k), 0.4);
    end
end
end

function testTrackerCoastsThroughTwoMissedFrames(testCase)
cfg = defaultConfig();
state = types.trackerState(cfg);
fused = oneFusedDetection(2.0, 0.0, cfg);
[tracked, state] = trackAgents(fused, state, 0.1, cfg);
fused = oneFusedDetection(2.1, 0.0, cfg);
[tracked, state] = trackAgents(fused, state, 0.1, cfg);
trackId = tracked.agents(1).id;
fused.count = 0;
[tracked, state] = trackAgents(fused, state, 0.1, cfg);
verifyEqual(testCase, tracked.count, 1);
[tracked, state] = trackAgents(fused, state, 0.1, cfg);
verifyEqual(testCase, tracked.count, 1);
verifyEqual(testCase, tracked.agents(1).id, trackId);
verifyGreaterThan(testCase, tracked.agents(1).confidence, 0);
verifyEqual(testCase, state.misses(1), 2);
end

function testGridMarksVehicleAndPothole(testCase)
cfg = defaultConfig();
scenario = createScenario('market', cfg, 5);
ego = types.egoState();
ego.x = -12;
ego.y = -3.1;
state = types.gridMapState(cfg);
fused = oneFusedDetection(0.0, 0.0, cfg);
fused.items(1).x = ego.x;
fused.items(1).y = ego.y;
fused.items(1).length = 3.0;
fused.items(1).width = 1.5;
fused.items(1).isStatic = true;
[map, state] = buildGridMap( ...
    scenario, fused, ego, state, 0.1, cfg);
centerRow = (cfg.grid.ny+1)/2;
centerColumn = (cfg.grid.nx+1)/2;
verifyGreaterThan(testCase, map.obstacleProb(centerRow, centerColumn), 0.9);
verifyLessThan(testCase, map.traversability(centerRow, centerColumn), 0.1);
verifyGreaterThan(testCase, ...
    map.surfaceCost(centerRow, centerColumn), 0.5);
verifyEqual(testCase, map.timestamp, scenario.time);
end

function testDynamicGridEvidenceDecays(testCase)
cfg = defaultConfig();
scenario = createScenario('market', cfg, 5);
ego = types.egoState();
state = types.gridMapState(cfg);
fused = oneFusedDetection(0.0, 0.0, cfg);
[firstMap, state] = buildGridMap( ...
    scenario, fused, ego, state, 0.1, cfg);
fused.count = 0;
[secondMap, ~] = buildGridMap( ...
    scenario, fused, ego, state, 1.0, cfg);
centerRow = (cfg.grid.ny+1)/2;
centerColumn = (cfg.grid.nx+1)/2;
verifyLessThan(testCase, secondMap.occupancy(centerRow, centerColumn), ...
    firstMap.occupancy(centerRow, centerColumn));
end

function testGridMapPlot(testCase)
visibility = get(groot, 'defaultFigureVisible');
visibilityCleanup = onCleanup(@() ...
    set(groot, 'defaultFigureVisible', visibility));
set(groot, 'defaultFigureVisible', 'off');
cfg = defaultConfig();
scenario = createScenario('market', cfg, 5);
state = types.gridMapState(cfg);
[map, ~] = buildGridMap(scenario, ...
    emptyFusedList(cfg), scenario.egoStart, state, 0.1, cfg);
figureHandle = figure('Visible', 'off');
figureCleanup = onCleanup(@() closeIfValid(figureHandle));
axesHandle = axes(figureHandle);
returnedAxes = plotGridMap(map, 'occupancy', axesHandle);
verifyEqual(testCase, returnedAxes, axesHandle);
verifyNotEmpty(testCase, findobj(axesHandle, 'Type', 'image'));
verifyError(testCase, @() plotGridMap(map, 'unknown', axesHandle), ...
    'plotGridMap:InvalidLayer');
clear figureCleanup visibilityCleanup
end

function cfg = deterministicSensorConfig()
cfg = defaultConfig();
cfg.perception.cameraDetectionBase = 1.0;
cfg.perception.cameraDetectionDecayM = 1000000.0;
cfg.perception.lidarDetectionProbability = 1.0;
cfg.perception.radarDetectionProbability = 1.0;
cfg.perception.cameraClassConfusionProb = 0.0;
cfg.perception.cameraFalsePositiveMean = 0.0;
cfg.perception.lidarFalsePositiveMean = 0.0;
cfg.perception.radarFalsePositiveMean = 0.0;
end

function list = oneFusedDetection(x, y, cfg)
list = emptyFusedList(cfg);
list.count = 1;
list.items(1).x = x;
list.items(1).y = y;
list.items(1).length = 4.0;
list.items(1).width = 2.0;
list.items(1).yaw = 0.0;
list.items(1).type = cfg.agentTypes.car;
list.items(1).confidence = 0.9;
list.items(1).positionVariance = 0.04;
end

function agent = makeTruthAgent(cfg, id, x)
agent = types.agentState(cfg);
agent.id = id;
agent.type = cfg.agentTypes.car;
agent.x = x;
agent.y = 0.0;
agent.yaw = 0.0;
agent.length = cfg.world.agentDimensions(agent.type, 1);
agent.width = cfg.world.agentDimensions(agent.type, 2);
agent.valid = true;
end

function list = emptyFusedList(cfg)
template = struct('x', 0.0, 'y', 0.0, ...
    'length', 0.0, 'width', 0.0, 'yaw', 0.0, 'type', 1, ...
    'confidence', 0.0, 'radialVelocity', 0.0, ...
    'radialUnitX', 1.0, 'radialUnitY', 0.0, ...
    'positionVariance', 1.0, 'radialVelocityVariance', 1.0, ...
    'hasCamera', false, 'hasLidar', false, 'hasRadar', false, ...
    'isStatic', false);
list = struct('count', 0, 'items', ...
    repmat(template, 1, cfg.perception.maxDetections));
end

function closeIfValid(handle)
if isgraphics(handle)
    close(handle);
end
end
