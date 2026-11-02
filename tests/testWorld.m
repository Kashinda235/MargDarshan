function tests = testWorld()
%TESTWORLD Unit tests for deterministic lightweight world simulation.
tests = functiontests(localfunctions);
end

function testScenariosAreDeterministic(testCase)
cfg = defaultConfig();
names = {'village','intersection','highwayMerge','market','cattleCrossing'};
for k = 1:numel(names)
    first = createScenario(names{k}, cfg, 42);
    second = createScenario(names{k}, cfg, 42);
    verifyEqual(testCase, first, second);
    verifyGreaterThan(testCase, first.road.drivableCount, 2);
    verifySize(testCase, first.agents.agents, [1 cfg.maxAgents]);
    if any(strcmp(names{k}, {'village','market'}))
        verifyEqual(testCase, first.road.laneMarkingCount, 0);
    else
        verifyGreaterThan(testCase, first.road.laneMarkingCount, 0);
    end
end
end

function testAgentEvolutionIsSeedDeterministic(testCase)
cfg = defaultConfig();
first = createScenario('market', cfg, 17);
second = createScenario('market', cfg, 17);
firstAgents = first.agents;
secondAgents = second.agents;
ego = first.egoStart;
for k = 1:25
    [firstAgents, ~, first] = stepWorld( ...
        first, firstAgents, ego, cfg.simulation.dt, cfg);
    [secondAgents, ~, second] = stepWorld( ...
        second, secondAgents, ego, cfg.simulation.dt, cfg);
end
verifyEqual(testCase, firstAgents, secondAgents);
verifyEqual(testCase, first.behaviors, second.behaviors);
end

function testBehaviorOutputAndCattleTrigger(testCase)
cfg = defaultConfig();
scenario = createScenario('cattleCrossing', cfg, 11);
[velocity, updated] = agentBehaviors( ...
    scenario, scenario.agents, scenario.egoStart, cfg.simulation.dt, cfg);
verifySize(testCase, velocity, [cfg.maxAgents 2]);
verifyTrue(testCase, updated.behaviors(1).triggered);
verifyGreaterThan(testCase, velocity(1,2), 0);
end

function testCollisionAndClearance(testCase)
cfg = defaultConfig();
ego = types.egoState();
ego.x = 0;
ego.y = 0;
ego.yaw = 0;
ego.v = 2;
agents = types.agentList(cfg);
agents.count = 2;
agents.agents(1) = makeAgent(cfg, 1, 0.5, 0, 0);
agents.agents(2) = makeAgent(cfg, 2, 10, 0, pi);
agents.agents(2).vx = -1;
obstacles = struct('count', 0, 'items', ...
    repmat(struct('id',0,'kind',cfg.world.obstacleKinds.parkedVehicle, ...
    'type',cfg.agentTypes.car,'x',0,'y',0, ...
    'yaw',0,'length',0,'width',0,'surfaceCost',0), ...
    1, cfg.world.maxStaticObstacles));
[collision, clearance, ttc] = checkCollision(ego, agents, obstacles, cfg);
verifyTrue(testCase, collision(1));
verifyEqual(testCase, clearance(1), 0);
verifyFalse(testCase, collision(2));
verifyGreaterThan(testCase, clearance(2), 0);
verifyTrue(testCase, isfinite(ttc(2)));
verifyGreaterThan(testCase, ttc(2), 0);
end

function testStaticObstacleCollision(testCase)
cfg = defaultConfig();
ego = types.egoState();
ego.x = 0;
ego.y = 0;
agents = types.agentList(cfg);
obstacles = struct('count', 1, 'items', ...
    repmat(struct('id',0,'kind',cfg.world.obstacleKinds.parkedVehicle, ...
    'type',cfg.agentTypes.car,'x',0,'y',0, ...
    'yaw',0,'length',0,'width',0,'surfaceCost',0), ...
    1, cfg.world.maxStaticObstacles));
obstacles.items(1).x = 0;
obstacles.items(1).y = 0;
obstacles.items(1).length = 2;
obstacles.items(1).width = 2;
[collision, clearance] = checkCollision(ego, agents, obstacles, cfg);
verifyTrue(testCase, collision);
verifyEqual(testCase, clearance, 0);
end

function testAgentSpeedLimitsAndGroundTruthGrid(testCase)
cfg = defaultConfig();
names = {'village','intersection','highwayMerge','market', ...
    'cattleCrossing'};
for scenarioIndex = 1:numel(names)
    scenario = createScenario(names{scenarioIndex}, cfg, 9);
    agents = scenario.agents;
    ego = scenario.egoStart;
    for k = 1:100
        [agents, ~, scenario] = stepWorld( ...
            scenario, agents, ego, cfg.simulation.dt, cfg);
        for n = 1:agents.count
            agent = agents.agents(n);
            verifyLessThanOrEqual(testCase, hypot(agent.vx, agent.vy), ...
                cfg.world.maxSpeeds(agent.type) + 1e-10);
        end
    end
end
map = groundTruthGrid(scenario, agents, ego, cfg);
verifySize(testCase, map.occupancy, [cfg.grid.ny cfg.grid.nx]);
verifyGreaterThanOrEqual(testCase, min(map.surfaceCost(:)), 0);
verifyLessThanOrEqual(testCase, max(map.surfaceCost(:)), 1);
verifyGreaterThanOrEqual(testCase, min(map.traversability(:)), 0);
verifyLessThanOrEqual(testCase, max(map.traversability(:)), 1);
verifyEqual(testCase, map.timestamp, scenario.time);
end

function testPlotWorldAndDemo(testCase)
visibility = get(groot, 'defaultFigureVisible');
visibilityCleanup = onCleanup(@() ...
    set(groot, 'defaultFigureVisible', visibility));
set(groot, 'defaultFigureVisible', 'off');
cfg = defaultConfig();
scenario = createScenario('village', cfg, 2);
figureHandle = figure('Visible', 'off');
figureCleanup = onCleanup(@() closeIfValid(figureHandle));
axesHandle = axes(figureHandle);
returnedAxes = plotWorld(scenario, scenario.agents, ...
    scenario.egoStart, axesHandle, cfg);
verifyEqual(testCase, returnedAxes, axesHandle);
verifyNotEmpty(testCase, findobj(axesHandle, 'Type', 'patch'));

cfg.world.demoDuration = cfg.world.demoDt;
cfg.world.demoPause = 0;
existingFigures = findall(groot, 'Type', 'figure');
demoWorld(cfg);
demoFigures = findall(groot, 'Type', 'figure');
for k = 1:numel(demoFigures)
    if ~any(demoFigures(k) == existingFigures)
        close(demoFigures(k));
    end
end
clear figureCleanup visibilityCleanup
end

function agent = makeAgent(cfg, id, x, y, yaw)
agent = types.agentState(cfg);
agent.id = id;
agent.type = cfg.agentTypes.car;
agent.x = x;
agent.y = y;
agent.yaw = yaw;
agent.length = cfg.world.agentDimensions(agent.type, 1);
agent.width = cfg.world.agentDimensions(agent.type, 2);
agent.valid = true;
end

function closeIfValid(handle)
if isgraphics(handle)
    close(handle);
end
end
