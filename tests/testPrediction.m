function tests = testPrediction()
%TESTPREDICTION Unit tests for interpretable agent prediction.
tests = functiontests(localfunctions);
end

function testStationaryConstantVelocityPrediction(testCase)
cfg = defaultConfig();
agents = types.agentList(cfg);
agents.count = 1;
agents.agents(1) = makeAgent(cfg, 1, cfg.agentTypes.car, 3, -2);
prediction = predictConstVelocity(agents, cfg);
verifyEqual(testCase, prediction.agents(1).x, ...
    repmat(3, cfg.prediction.steps, 1));
verifyEqual(testCase, prediction.agents(1).y, ...
    repmat(-2, cfg.prediction.steps, 1));
verifyGreaterThan(testCase, prediction.agents(1).sigma(end), ...
    prediction.agents(1).sigma(1));
end

function testSeparationPointsAwayFromCloseNeighbour(testCase)
cfg = defaultConfig();
agents = types.agentList(cfg);
agents.count = 2;
agents.agents(1) = makeAgent(cfg, 1, cfg.agentTypes.car, 0, 0);
agents.agents(2) = makeAgent(cfg, 2, cfg.agentTypes.car, 1, 0);
ego = types.egoState();
ego.x = -100;
forces = boidForces(agents.agents(1), agents, ego, [], cfg);
verifyLessThan(testCase, forces.separation(1), 0);
verifyEqual(testCase, forces.separation(2), 0, 'AbsTol', 1e-12);
ego.x = 1;
egoForces = boidForces(agents.agents(1), ...
    types.agentList(cfg), ego, [], cfg);
verifyLessThan(testCase, egoForces.separation(1), 0);
agent = agents.agents(1);
agent.vy = 1;
flowForces = boidForces(agent, types.agentList(cfg), ego, [], cfg);
verifyLessThan(testCase, flowForces.goalAttraction(2), 0);
agent.y = -9.2;
road = [-10 -10; 10 -10; 10 10; -10 10];
boundaryForces = boidForces(agent, types.agentList(cfg), ...
    ego, road, cfg);
verifyGreaterThan(testCase, boundaryForces.roadBoundary(2), 0);
end

function testTypeHypothesesAndProbabilities(testCase)
cfg = defaultConfig();
agents = types.agentList(cfg);
agents.count = 2;
agents.agents(1) = makeAgent(cfg, 1, cfg.agentTypes.truck, 0, 0);
agents.agents(2) = makeAgent(cfg, 2, cfg.agentTypes.motorcycle, 8, 0);
ego = types.egoState();
truck = agentTypeModel(cfg, cfg.agentTypes.truck);
motorcycle = agentTypeModel(cfg, cfg.agentTypes.motorcycle);
verifyLessThan(testCase, truck.hypothesisCount, ...
    motorcycle.hypothesisCount);
predictions = predictAgentsBoid(agents, ego, [], cfg);
for k = 1:predictions.count
    prediction = predictions.agents(k);
    verifyEqual(testCase, sum(prediction.modeProbability), 1, ...
        'AbsTol', 1e-12);
    verifyEqual(testCase, prediction.hypothesisCount, ...
        agentTypeModel(cfg, prediction.type).hypothesisCount);
end
end

function testPredictorsRespectConfiguredSpeedLimit(testCase)
cfg = defaultConfig();
agents = types.agentList(cfg);
agents.count = 1;
agents.agents(1) = makeAgent(cfg, 1, cfg.agentTypes.car, 0, 0);
agents.agents(1).vx = 100;
ego = types.egoState();
model = agentTypeModel(cfg, cfg.agentTypes.car);
cv = predictConstVelocity(agents, cfg);
boid = predictAgentsBoid(agents, ego, [], cfg);
for predictionIndex = 1:2
    if predictionIndex == 1
        prediction = cv.agents(1);
    else
        prediction = boid.agents(1);
    end
    for mode = 1:prediction.hypothesisCount
        stepDistance = hypot( ...
            diff(prediction.hypothesisX(:, mode)), ...
            diff(prediction.hypothesisY(:, mode)));
        verifyLessThanOrEqual(testCase, max(stepDistance), ...
            model.maxSpeedMps*cfg.prediction.dt + 1e-10);
    end
end
end

function testRiskMapPeaksNearPredictedPosition(testCase)
cfg = defaultConfig();
agents = types.agentList(cfg);
agents.count = 1;
agents.agents(1) = makeAgent(cfg, 1, cfg.agentTypes.car, 0, 0);
prediction = predictConstVelocity(agents, cfg);
ego = types.egoState();
[risk, localX, localY] = predictionRiskMap(prediction, ego, cfg);
verifySize(testCase, risk, ...
    [cfg.grid.nx cfg.grid.ny cfg.prediction.steps]);
[~, centerIndex] = min(abs(localX(:, 1)));
[~, rowIndex] = min(abs(localY(1, :)));
centerRisk = risk(centerIndex, rowIndex, 1);
farRisk = risk(1, 1, 1);
verifyGreaterThan(testCase, centerRisk, farRisk);
end

function testPredictionDispatcherAndDemo(testCase)
cfg = defaultConfig();
agents = types.agentList(cfg);
agents.count = 1;
agents.agents(1) = makeAgent(cfg, 1, cfg.agentTypes.car, 0, 0);
ego = types.egoState();
cfg.prediction.method = 'cv';
cv = predictAgents(agents, ego, [], cfg);
verifyEqual(testCase, cv.agents(1).hypothesisCount, 1);
cfg.prediction.method = 'nn';
verifyError(testCase, @() predictAgents(agents, ego, [], cfg), ...
    'predictAgents:UnavailableMethod');
cfg.prediction.method = 'boid';
cfg.world.demoDuration = cfg.prediction.horizon + cfg.world.demoDt;
cfg.world.demoPause = 0;
oldVisibility = get(groot, 'defaultFigureVisible');
visibilityCleanup = onCleanup(@() ...
    set(groot, 'defaultFigureVisible', oldVisibility));
set(groot, 'defaultFigureVisible', 'off');
existingFigures = findall(groot, 'Type', 'figure');
metrics = demoPrediction(cfg);
verifyTrue(testCase, isfinite(metrics.cvADE));
verifyTrue(testCase, isfinite(metrics.boidADE));
verifyTrue(testCase, isfinite(metrics.cvFDE));
verifyTrue(testCase, isfinite(metrics.boidFDE));
newFigures = setdiff(findall(groot, 'Type', 'figure'), existingFigures);
for k = 1:numel(newFigures)
    close(newFigures(k));
end
clear visibilityCleanup
end

function agent = makeAgent(cfg, id, type, x, y)
agent = types.agentState(cfg);
agent.id = id;
agent.type = type;
agent.x = x;
agent.y = y;
agent.length = cfg.world.agentDimensions(type, 1);
agent.width = cfg.world.agentDimensions(type, 2);
agent.confidence = 1;
agent.valid = true;
end
