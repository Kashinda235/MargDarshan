function log = runSimulation(scenarioName, cfg, seed, options)
%RUNSIMULATION Run one fixed-step closed-loop scenario.
% Inputs: scenarioName, a createScenario name; cfg, defaultConfig output;
% seed, nonnegative integer; options.planner is 'margdarshan' or 'baseline',
% options.durationS is seconds, options.measureTiming is logical,
% options.initialScenario can provide a randomized scene, and options.ablation
% enables non-default component switches used by metrics/ablationStudy.
% Outputs: preallocated-and-trimmed log with SI states, truth agents,
% decisions, pocket/trajectory samples, collisions, termination, and
% per-module timing in seconds. Example:
% log = runSimulation('village',defaultConfig(),7,struct('planner','baseline'));

if nargin < 2 || isempty(cfg)
    cfg = defaultConfig();
end
if nargin < 3 || isempty(seed)
    seed = 1;
end
if nargin < 4 || isempty(options)
    options = struct();
end
validateattributes(seed, {'numeric'}, ...
    {'scalar','real','finite','integer','nonnegative'}, mfilename, 'seed');
if ~isstruct(options) || ~isscalar(options)
    error('runSimulation:InvalidOptions', 'options must be a scalar struct.');
end
planner = optionValue(options, 'planner', 'margdarshan');
if isstring(planner) && isscalar(planner)
    planner = char(planner);
end
if ~ischar(planner) || ~any(strcmpi(planner, {'margdarshan','baseline'}))
    error('runSimulation:InvalidPlanner', ...
        'options.planner must be ''margdarshan'' or ''baseline''.');
end
planner = lower(planner);
durationS = optionValue(options, 'durationS', cfg.simulation.duration);
measureTiming = optionValue(options, 'measureTiming', true);
initialScenario = optionValue(options, 'initialScenario', []);
ablation = optionValue(options, 'ablation', struct());
validateattributes(durationS, {'numeric'}, ...
    {'scalar','real','finite','positive'}, mfilename, 'options.durationS');
if ~(islogical(measureTiming) && isscalar(measureTiming))
    error('runSimulation:InvalidTimingOption', ...
        'options.measureTiming must be a logical scalar.');
end
ablation = validateAblation(ablation);

if isempty(initialScenario)
    scenario = createScenario(scenarioName, cfg, seed);
else
    if ~isstruct(initialScenario) || ~isfield(initialScenario, 'name') ...
            || ~strcmp(char(initialScenario.name), char(scenarioName))
        error('runSimulation:InvalidInitialScenario', ...
            'options.initialScenario must be a scenario for scenarioName.');
    end
    scenario = initialScenario;
end
if ~isempty(ablation.predictionMethod)
    cfg.prediction.method = ablation.predictionMethod;
end
ego = scenario.egoStart;
route = makeRoute(scenario, cfg);
maximumSteps = ceil(durationS/cfg.simDt);
log = allocateLog(maximumSteps+1, cfg, scenario, route, planner, seed);
log.time(1) = scenario.time;
log = storeAgents(log, 1, scenario.agents);
log.ego = storeEgo(log.ego, 1, ego);

[initialHits, initialClearances, initialTTcs] = checkCollision( ...
    ego, scenario.agents, scenario.obstacles, cfg);
if ~isempty(initialClearances)
    log.minimumClearanceM(1) = min(initialClearances);
end
if ~isempty(initialTTcs)
    log.minimumTTCS(1) = min(initialTTcs);
end
if any(initialHits)
    log.collision(1) = true;
    log.terminationReason = 'collision';
elseif norm(egoPosition(ego)-scenario.goal) ...
        <= cfg.simulation.goalToleranceM
    log.completed = true;
    log.terminationReason = 'goal';
end

tracker = types.trackerState(cfg);
gridState = types.gridMapState(cfg);
sensorRandomState = seed;
controllerState = [];
previousDecision = [];
decision = types.decision(cfg);
diagnostics = struct('pocketState', types.pocket(cfg));
selectedTrajectory = types.trajectory(cfg);
trajectoryValid = false;
routeIndex = 1;
nextReplanTime = -inf;
lastEmergencyBrake = false;
stuckAnchor = egoPosition(ego);
stuckTime = 0.0;
sampleCount = 1;

for step = 1:maximumSteps
    if ~isempty(log.terminationReason)
        break;
    end
    timing = emptyTiming();

    timer = tic;
    [truth, staticObstacles, scenario] = stepWorld( ...
        scenario, scenario.agents, ego, cfg.simDt, cfg);
    timing.world = toc(timer);

    timer = tic;
    [frame, sensorRandomState] = simulateSensors( ...
        scenario, truth, ego, cfg, sensorRandomState);
    timing.sensors = toc(timer);

    timer = tic;
    detections = fuseDetections(frame, cfg);
    timing.fusion = toc(timer);

    timer = tic;
    [trackedAgents, tracker] = trackAgents( ...
        detections, tracker, cfg.simDt, cfg);
    timing.tracking = toc(timer);

    timer = tic;
    [gridMap, gridState] = buildGridMap( ...
        scenario, detections, ego, gridState, cfg.simDt, cfg);
    timing.gridMap = toc(timer);

    timer = tic;
    [routeIndex, reference] = getLocalReference( ...
        ego, route, routeIndex, cfg);
    timing.reference = toc(timer);

    predictions = [];
    if strcmp(planner, 'margdarshan')
        timer = tic;
        predictions = predictAgents(trackedAgents, ego, ...
            scenario.road.boundaries(1:scenario.road.boundaryCount,:), cfg);
        timing.prediction = toc(timer);
    end

    isReplan = scenario.time+eps(scenario.time) >= nextReplanTime;
    if isReplan
        timer = tic;
        if strcmp(planner, 'margdarshan')
            [decision, diagnostics] = planLocal(ego, reference, ...
                gridMap, trackedAgents, predictions, previousDecision, cfg, ...
                ablation);
        else
            [decision, diagnostics] = baselineLaneFollower( ...
                ego, reference, trackedAgents, cfg);
            diagnostics.pocketState = types.pocket(cfg);
        end
        timing.planning = toc(timer);
        selectedTrajectory = decision.selectedTrajectory;
        trajectoryValid = true;
        previousDecision = decision;
        nextReplanTime = scenario.time+decision.replanPeriod;
    end
    if ~trajectoryValid
        error('runSimulation:MissingTrajectory', ...
            'The selected planner did not provide an initial trajectory.');
    end

    timer = tic;
    [command, controllerState] = trackTrajectory( ...
        ego, selectedTrajectory, controllerState, cfg);
    timing.control = toc(timer);

    timer = tic;
    ego = vehicleModel(ego, command, cfg);
    timing.vehicle = toc(timer);

    timer = tic;
    [collisions, clearances, ttcs] = checkCollision( ...
        ego, truth, staticObstacles, cfg);
    timing.collision = toc(timer);
    timing.replanLatency = sum([timing.sensors timing.fusion ...
        timing.tracking timing.gridMap timing.reference ...
        timing.prediction timing.planning]);
    if ~measureTiming
        timing = emptyTiming();
    end

    sampleCount = sampleCount+1;
    log.time(sampleCount) = scenario.time;
    log.ego = storeEgo(log.ego, sampleCount, ego);
    log = storeAgents(log, sampleCount, truth);
    log.collision(sampleCount) = any(collisions);
    if isempty(clearances)
        log.minimumClearanceM(sampleCount) = inf;
    else
        log.minimumClearanceM(sampleCount) = min(clearances);
    end
    if isempty(ttcs)
        log.minimumTTCS(sampleCount) = inf;
    else
        log.minimumTTCS(sampleCount) = min(ttcs);
    end
    log.decisionMode(sampleCount) = decision.mode;
    log.cost(sampleCount) = decision.selectedTrajectory.cost;
    log.fallbackCode(sampleCount) = decision.fallbackCode;
    log.replanned(sampleCount) = isReplan;
    log.emergencyBrake(sampleCount) = ...
        command.accel <= -cfg.metrics.emergencyBrakeThresholdMps2;
    log.emergencyBrakeEvent(sampleCount) = ...
        log.emergencyBrake(sampleCount) && ~lastEmergencyBrake;
    lastEmergencyBrake = log.emergencyBrake(sampleCount);
    log.command.accel(sampleCount) = command.accel;
    log.command.steer(sampleCount) = command.steer;
    log.command.brake(sampleCount) = command.brake;
    log = storeTrajectory(log, sampleCount, selectedTrajectory);
    log.trajectoryValid(sampleCount) = true;
    log = storePocket(log, sampleCount, diagnostics.pocketState);
    log.timing = storeTiming(log.timing, sampleCount, timing);

    if log.collision(sampleCount)
        log.terminationReason = 'collision';
    elseif norm(egoPosition(ego)-scenario.goal) ...
            <= cfg.simulation.goalToleranceM
        log.completed = true;
        log.terminationReason = 'goal';
    else
        if ego.v <= cfg.simulation.stuckSpeedMps ...
                && norm(egoPosition(ego)-stuckAnchor) ...
                < cfg.simulation.stuckDistanceM
            stuckTime = stuckTime+cfg.simDt;
        else
            stuckAnchor = egoPosition(ego);
            stuckTime = 0.0;
        end
        if stuckTime >= cfg.simulation.stuckDurationS
            log.terminationReason = 'stuck';
        elseif scenario.time+eps(scenario.time) >= durationS
            log.terminationReason = 'timeout';
        end
    end
end

if isempty(log.terminationReason)
    log.terminationReason = 'timeout';
end
log = trimLog(log, sampleCount);
log.simulatedDurationS = log.time(end)-log.time(1);
log.terminationReason = char(log.terminationReason);
end

function route = makeRoute(scenario, cfg)
start = [scenario.egoStart.x scenario.egoStart.y];
delta = scenario.goal-start;
distance = hypot(delta(1), delta(2));
if distance <= eps
    direction = [cos(scenario.egoStart.yaw) sin(scenario.egoStart.yaw)];
else
    direction = delta/distance;
end
extension = max(cfg.nav.referencePointCount, 2)*cfg.nav.routeSpacing;
waypoints = [start; scenario.goal; scenario.goal+extension*direction];
route = densifyRoute(waypoints, cfg);
end

function log = allocateLog(sampleCapacity, cfg, scenario, route, planner, seed)
log = struct();
log.scenarioName = scenario.name;
log.seed = seed;
log.planner = planner;
log.goal = scenario.goal;
log.route = route;
log.time = zeros(1, sampleCapacity);
log.ego = struct( ...
    'x', zeros(1,sampleCapacity), 'y', zeros(1,sampleCapacity), ...
    'yaw', zeros(1,sampleCapacity), 'v', zeros(1,sampleCapacity), ...
    'a', zeros(1,sampleCapacity), 'steer', zeros(1,sampleCapacity), ...
    'yawRate', zeros(1,sampleCapacity));
log.agents = struct( ...
    'x', zeros(cfg.maxAgents,sampleCapacity), ...
    'y', zeros(cfg.maxAgents,sampleCapacity), ...
    'yaw', zeros(cfg.maxAgents,sampleCapacity), ...
    'vx', zeros(cfg.maxAgents,sampleCapacity), ...
    'vy', zeros(cfg.maxAgents,sampleCapacity), ...
    'length', zeros(cfg.maxAgents,sampleCapacity), ...
    'width', zeros(cfg.maxAgents,sampleCapacity), ...
    'type', zeros(cfg.maxAgents,sampleCapacity), ...
    'valid', false(cfg.maxAgents,sampleCapacity));
log.trajectory = struct( ...
    'x', zeros(cfg.planner.samples,sampleCapacity), ...
    'y', zeros(cfg.planner.samples,sampleCapacity), ...
    'yaw', zeros(cfg.planner.samples,sampleCapacity), ...
    'v', zeros(cfg.planner.samples,sampleCapacity));
log.trajectoryValid = false(1,sampleCapacity);
log.pocket = struct( ...
    'boundaryX', zeros(cfg.planner.maxBoundaryVertices,sampleCapacity), ...
    'boundaryY', zeros(cfg.planner.maxBoundaryVertices,sampleCapacity), ...
    'boundaryCount', zeros(1,sampleCapacity), ...
    'areaM2', zeros(1,sampleCapacity), ...
    'forwardDepthM', zeros(1,sampleCapacity));
log.collision = false(1,sampleCapacity);
log.minimumClearanceM = inf(1,sampleCapacity);
log.minimumTTCS = inf(1,sampleCapacity);
log.decisionMode = zeros(1,sampleCapacity);
log.cost = zeros(1,sampleCapacity);
log.fallbackCode = zeros(1,sampleCapacity);
log.replanned = false(1,sampleCapacity);
log.emergencyBrake = false(1,sampleCapacity);
log.emergencyBrakeEvent = false(1,sampleCapacity);
log.command = struct( ...
    'accel', zeros(1,sampleCapacity), ...
    'steer', zeros(1,sampleCapacity), ...
    'brake', zeros(1,sampleCapacity));
log.timing = allocateTiming(sampleCapacity);
log.completed = false;
log.terminationReason = '';
log.simulatedDurationS = 0.0;
end

function timing = allocateTiming(sampleCapacity)
template = emptyTiming();
fields = fieldnames(template);
timing = struct();
for k = 1:numel(fields)
    timing.(fields{k}) = zeros(1,sampleCapacity);
end
end

function timing = emptyTiming()
timing = struct( ...
    'world', 0.0, 'sensors', 0.0, 'fusion', 0.0, ...
    'tracking', 0.0, 'gridMap', 0.0, 'reference', 0.0, ...
    'prediction', 0.0, 'planning', 0.0, 'control', 0.0, ...
    'vehicle', 0.0, 'collision', 0.0, 'replanLatency', 0.0);
end

function log = storeAgents(log, sample, agents)
for k = 1:agents.count
    agent = agents.agents(k);
    log.agents.x(k,sample) = agent.x;
    log.agents.y(k,sample) = agent.y;
    log.agents.yaw(k,sample) = agent.yaw;
    log.agents.vx(k,sample) = agent.vx;
    log.agents.vy(k,sample) = agent.vy;
    log.agents.length(k,sample) = agent.length;
    log.agents.width(k,sample) = agent.width;
    log.agents.type(k,sample) = agent.type;
    log.agents.valid(k,sample) = agent.valid;
end
end

function state = storeEgo(state, sample, ego)
state.x(sample) = ego.x;
state.y(sample) = ego.y;
state.yaw(sample) = ego.yaw;
state.v(sample) = ego.v;
state.a(sample) = ego.a;
state.steer(sample) = ego.steer;
state.yawRate(sample) = ego.yawRate;
end

function log = storeTrajectory(log, sample, path)
log.trajectory.x(:,sample) = path.x(:);
log.trajectory.y(:,sample) = path.y(:);
log.trajectory.yaw(:,sample) = path.yaw(:);
log.trajectory.v(:,sample) = path.v(:);
end

function log = storePocket(log, sample, pocketState)
count = pocketState.boundaryCount;
if count > 0
    log.pocket.boundaryX(1:count,sample) = pocketState.boundary(1:count,1);
    log.pocket.boundaryY(1:count,sample) = pocketState.boundary(1:count,2);
end
log.pocket.boundaryCount(sample) = count;
log.pocket.areaM2(sample) = pocketState.area;
log.pocket.forwardDepthM(sample) = pocketState.forwardDepth;
end

function timingLog = storeTiming(timingLog, sample, timing)
fields = fieldnames(timing);
for k = 1:numel(fields)
    timingLog.(fields{k})(sample) = timing.(fields{k});
end
end

function log = trimLog(log, sampleCount)
log.time = log.time(1:sampleCount);
egoFields = fieldnames(log.ego);
for k = 1:numel(egoFields)
    log.ego.(egoFields{k}) = log.ego.(egoFields{k})(1:sampleCount);
end
agentFields = fieldnames(log.agents);
for k = 1:numel(agentFields)
    log.agents.(agentFields{k}) = log.agents.(agentFields{k})(:,1:sampleCount);
end
trajectoryFields = fieldnames(log.trajectory);
for k = 1:numel(trajectoryFields)
    log.trajectory.(trajectoryFields{k}) = ...
        log.trajectory.(trajectoryFields{k})(:,1:sampleCount);
end
log.trajectoryValid = log.trajectoryValid(1:sampleCount);
pocketFields = fieldnames(log.pocket);
for k = 1:numel(pocketFields)
    log.pocket.(pocketFields{k}) = log.pocket.(pocketFields{k})(:,1:sampleCount);
end
sampleFields = {'collision','minimumClearanceM','minimumTTCS', ...
    'decisionMode','cost','fallbackCode','replanned','emergencyBrake', ...
    'emergencyBrakeEvent'};
for k = 1:numel(sampleFields)
    log.(sampleFields{k}) = log.(sampleFields{k})(1:sampleCount);
end
commandFields = fieldnames(log.command);
for k = 1:numel(commandFields)
    log.command.(commandFields{k}) = ...
        log.command.(commandFields{k})(1:sampleCount);
end
timingFields = fieldnames(log.timing);
for k = 1:numel(timingFields)
    log.timing.(timingFields{k}) = log.timing.(timingFields{k})(1:sampleCount);
end
end

function value = optionValue(options, name, defaultValue)
if isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end

function ablation = validateAblation(value)
fields = {'disableAdaptiveConfig','disablePocketGating', ...
    'disableTrafficRiskCost'};
if ~isstruct(value) || ~isscalar(value)
    error('runSimulation:InvalidAblation', ...
        'options.ablation must be a scalar struct.');
end
ablation = struct( ...
    'disableAdaptiveConfig', false, ...
    'disablePocketGating', false, ...
    'disableTrafficRiskCost', false, ...
    'predictionMethod', '');
allowedFields = [fields {'predictionMethod'}];
unknownFields = setdiff(fieldnames(value), allowedFields);
if ~isempty(unknownFields)
    error('runSimulation:InvalidAblation', ...
        'Unknown ablation field "%s".', unknownFields{1});
end
if isfield(value, 'disableAdaptiveConfig')
    ablation.disableAdaptiveConfig = validateAblationFlag( ...
        value.disableAdaptiveConfig);
end
if isfield(value, 'disablePocketGating')
    ablation.disablePocketGating = validateAblationFlag( ...
        value.disablePocketGating);
end
if isfield(value, 'disableTrafficRiskCost')
    ablation.disableTrafficRiskCost = validateAblationFlag( ...
        value.disableTrafficRiskCost);
end
if isfield(value, 'predictionMethod')
    method = value.predictionMethod;
    if isstring(method) && isscalar(method)
        method = char(method);
    end
    if ~ischar(method) || ~any(strcmpi(method, {'boid','cv'}))
        error('runSimulation:InvalidAblation', ...
            'ablation.predictionMethod must be ''boid'' or ''cv''.');
    end
    ablation.predictionMethod = lower(method);
end
end

function flag = validateAblationFlag(flag)
if ~(islogical(flag) && isscalar(flag))
    error('runSimulation:InvalidAblation', ...
        'Each ablation switch must be a logical scalar.');
end
end

function position = egoPosition(ego)
position = [ego.x ego.y];
end
