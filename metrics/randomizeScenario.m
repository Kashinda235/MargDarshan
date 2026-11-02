function scenario = randomizeScenario(scenario, cfg, seed)
%RANDOMIZESCENARIO Apply deterministic traffic and road-condition variation.
% Inputs: scenario from createScenario; cfg from defaultConfig; seed, a
% nonnegative integer. SI positions, dimensions, and speeds are preserved.
% Outputs: scenario with randomized traffic density/types, lane-marking
% availability, parked vehicles, and potholes when supported.
% Example: s = randomizeScenario(createScenario('village',cfg,7),cfg,7);

if nargin < 2 || isempty(cfg)
    cfg = defaultConfig();
end
if nargin < 3
    seed = scenario.seed;
end
validateattributes(seed, {'numeric'}, ...
    {'scalar','real','finite','integer','nonnegative'}, ...
    mfilename, 'seed');
if ~isstruct(scenario) || ~all(isfield(scenario, ...
        {'name','agents','behaviors','road','obstacles','potholes', ...
        'egoStart','goal'}))
    error('randomizeScenario:InvalidScenario', ...
        'scenario must be a createScenario result.');
end

stream = RandStream('mt19937ar', 'Seed', double(seed));
scenario.seed = double(seed);
scenario = varyTraffic(scenario, cfg, stream);
scenario = varyLaneMarkings(scenario, stream);
scenario = varyParkedVehicles(scenario, cfg, stream);
scenario = varyPotholes(scenario, stream);
end

function scenario = varyTraffic(scenario, cfg, stream)
originalCount = scenario.agents.count;
if originalCount == 0
    return;
end
minimumCount = max(1, floor(0.6*originalCount));
maximumCount = min(cfg.maxAgents, originalCount ...
    + max(1, ceil(0.5*originalCount)));
targetCount = randi(stream, [minimumCount maximumCount]);
scenario.agents.count = min(originalCount, targetCount);
for k = 1:scenario.agents.count
    if rand(stream) < 0.4
        scenario.agents.agents(k) = varyAgentType( ...
            scenario.agents.agents(k), cfg, stream);
    end
end
for k = originalCount+1:targetCount
    sourceIndex = randi(stream, originalCount);
    agent = scenario.agents.agents(sourceIndex);
    behavior = scenario.behaviors(sourceIndex);
    agent.id = k;
    agent = varyAgentType(agent, cfg, stream);
    [agent.x, agent.y, agent.yaw] = trafficPosition( ...
        scenario, stream);
    speed = (0.25+0.55*rand(stream))*cfg.world.maxSpeeds(agent.type);
    agent.vx = speed*cos(agent.yaw);
    agent.vy = speed*sin(agent.yaw);
    agent.ax = 0.0;
    agent.ay = 0.0;
    agent.valid = true;
    behavior.model = cfg.world.behaviorModels.straight;
    behavior.speed = speed;
    behavior.laneY = agent.y;
    behavior.jitter = 0.0;
    behavior.phase = 0.0;
    behavior.targetX = agent.x;
    behavior.targetY = agent.y;
    behavior.triggered = false;
    behavior.goalChanges = 0;
    behavior.direction = sign(cos(agent.yaw));
    if behavior.direction == 0
        behavior.direction = 1.0;
    end
    scenario.agents.agents(k) = agent;
    scenario.behaviors(k) = behavior;
end
scenario.agents.count = targetCount;
end

function agent = varyAgentType(agent, cfg, stream)
agent.type = randi(stream, size(cfg.world.agentDimensions, 1));
agent.length = cfg.world.agentDimensions(agent.type, 1);
agent.width = cfg.world.agentDimensions(agent.type, 2);
speed = hypot(agent.vx, agent.vy);
speed = min(speed, cfg.world.maxSpeeds(agent.type));
agent.vx = speed*cos(agent.yaw);
agent.vy = speed*sin(agent.yaw);
end

function [x, y, yaw] = trafficPosition(scenario, stream)
start = [scenario.egoStart.x scenario.egoStart.y];
delta = scenario.goal-start;
distance = hypot(delta(1), delta(2));
direction = delta/max(distance, eps);
lateralLimit = min(3.0, max(1.0, ...
    0.25*(max(scenario.road.boundaries( ...
    1:scenario.road.boundaryCount, 2))-min(scenario.road.boundaries( ...
    1:scenario.road.boundaryCount, 2)))));
lateral = (2*rand(stream)-1)*lateralLimit;
position = start+(0.2+0.65*rand(stream))*delta ...
    + lateral*[-direction(2) direction(1)];
x = position(1);
y = position(2);
if rand(stream) < 0.5
    yaw = atan2(direction(2), direction(1));
else
    yaw = atan2(-direction(2), -direction(1));
end
end

function scenario = varyLaneMarkings(scenario, stream)
count = scenario.road.laneMarkingCount;
if count == 0
    return;
end
retainedCount = randi(stream, [0 count]);
if retainedCount > 0
    order = randperm(stream, count);
    scenario.road.laneMarkings(1:retainedCount,:) = ...
        scenario.road.laneMarkings(order(1:retainedCount),:);
end
scenario.road.laneMarkingCount = retainedCount;
end

function scenario = varyParkedVehicles(scenario, cfg, stream)
items = scenario.obstacles.items;
writeIndex = 0;
for k = 1:scenario.obstacles.count
    item = items(k);
    if item.kind == cfg.world.obstacleKinds.parkedVehicle
        if rand(stream) < 0.3
            continue;
        end
        item.x = item.x+(2*rand(stream)-1)*1.5;
        item.y = item.y+(2*rand(stream)-1)*0.5;
    end
    writeIndex = writeIndex+1;
    items(writeIndex) = item;
end
if ~strcmp(scenario.name, 'cattleCrossing') && ...
        writeIndex < cfg.world.maxStaticObstacles
    additionalCount = randi(stream, [0 2]);
    for k = 1:min(additionalCount, cfg.world.maxStaticObstacles-writeIndex)
        item = items(1);
        item.id = writeIndex+1;
        item.kind = cfg.world.obstacleKinds.parkedVehicle;
        item.type = randi(stream, 5);
        item.length = cfg.world.agentDimensions(item.type, 1);
        item.width = cfg.world.agentDimensions(item.type, 2);
        [item.x, item.y, item.yaw] = trafficPosition(scenario, stream);
        item.y = item.y+sign(rand(stream)-0.5)*3.5;
        item.surfaceCost = 0.0;
        writeIndex = writeIndex+1;
        items(writeIndex) = item;
    end
end
scenario.obstacles.items = items;
scenario.obstacles.count = writeIndex;
end

function scenario = varyPotholes(scenario, stream)
count = scenario.potholes.count;
if count == 0
    return;
end
items = scenario.potholes.items;
writeIndex = 0;
for k = 1:count
    if rand(stream) < 0.2
        continue;
    end
    item = items(k);
    item.x = item.x+(2*rand(stream)-1)*2.0;
    item.y = item.y+(2*rand(stream)-1)*0.8;
    item.radius = item.radius*(0.8+0.4*rand(stream));
    item.cost = min(1.0, max(0.0, ...
        item.cost*(0.75+0.5*rand(stream))));
    writeIndex = writeIndex+1;
    items(writeIndex) = item;
end
scenario.potholes.items = items;
scenario.potholes.count = writeIndex;
end
