function tests = testTypes()
%TESTTYPES Unit tests for MargDarshan data-contract constructors.
tests = functiontests(localfunctions);
end

function testEgoState(testCase)
state = types.egoState();
verifyFields(testCase, state, {'x','y','yaw','v','a','steer','yawRate'});
end

function testAgentState(testCase)
agent = types.agentState();
verifyFields(testCase, agent, ...
    {'id','type','x','y','vx','vy','ax','ay','yaw','length','width', ...
     'confidence','valid'});
verifyClass(testCase, agent.valid, 'logical');
end

function testAgentList(testCase)
cfg = defaultConfig();
list = types.agentList(cfg);
verifyFields(testCase, list, {'count','agents'});
verifyEqual(testCase, numel(list.agents), cfg.maxAgents);
verifyFields(testCase, list.agents(1), ...
    {'id','type','x','y','vx','vy','ax','ay','yaw','length','width', ...
     'confidence','valid'});
end

function testGridMap(testCase)
cfg = defaultConfig();
map = types.gridMap(cfg);
verifyFields(testCase, map, ...
    {'resolution','originX','originY','nx','ny','occupancy','height', ...
     'obstacleProb','traversability','surfaceCost','timestamp'});
verifySize(testCase, map.occupancy, [cfg.grid.ny cfg.grid.nx]);
verifySize(testCase, map.height, [cfg.grid.ny cfg.grid.nx]);
verifyEqual(testCase, map.originX, ...
    -((cfg.grid.nx-1)/2)*cfg.grid.resolution);
verifyEqual(testCase, map.originY, ...
    -((cfg.grid.ny-1)/2)*cfg.grid.resolution);
end

function testPredictedAgent(testCase)
cfg = defaultConfig();
prediction = types.predictedAgent(cfg);
verifyFields(testCase, prediction, ...
    {'id','type','length','width','confidence','T','t','x','y','sigma', ...
     'hypothesisCount','hypothesisX','hypothesisY','hypothesisYaw', ...
     'hypothesisSigma','modeProbability'});
verifyEqual(testCase, prediction.T, cfg.prediction.steps);
verifySize(testCase, prediction.x, [cfg.prediction.steps 1]);
verifySize(testCase, prediction.hypothesisX, ...
    [cfg.prediction.steps cfg.prediction.modes]);
verifySize(testCase, prediction.modeProbability, [1 cfg.prediction.modes]);
end

function testPocket(testCase)
cfg = defaultConfig();
pocketState = types.pocket(cfg);
verifyFields(testCase, pocketState, ...
    {'mask','boundary','boundaryCount','area','forwardDepth', ...
     'lateralLeft','lateralRight','riskThreshold','quality', ...
     'isEmpty','isSmall'});
verifyClass(testCase, pocketState.mask, 'logical');
verifySize(testCase, pocketState.mask, [cfg.grid.ny cfg.grid.nx]);
verifySize(testCase, pocketState.boundary, ...
    [cfg.planner.maxBoundaryVertices 2]);
verifyClass(testCase, pocketState.isEmpty, 'logical');
verifyClass(testCase, pocketState.isSmall, 'logical');
end

function testTrajectory(testCase)
cfg = defaultConfig();
path = types.trajectory(cfg);
verifyFields(testCase, path, ...
    {'t','x','y','yaw','v','kappa','cost','costBreakdown','feasible'});
verifySize(testCase, path.t, [cfg.planner.samples 1]);
verifyFields(testCase, path.costBreakdown, ...
    {'progress','safety','comfort','smoothness','surface'});
end

function testDecision(testCase)
cfg = defaultConfig();
result = types.decision(cfg);
verifyFields(testCase, result, ...
    {'selectedTrajectory','targetSpeed','safetyMargin','horizon', ...
     'replanPeriod','mode'});
verifyFields(testCase, result.selectedTrajectory, ...
    {'t','x','y','yaw','v','kappa','cost','costBreakdown','feasible'});
end

function verifyFields(testCase, value, expected)
actual = fieldnames(value);
verifyEqual(testCase, sort(actual), sort(expected(:)));
end
