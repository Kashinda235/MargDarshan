function metrics = demoPocket(cfg)
%DEMOPOCKET Animate safe-pocket evolution in market and cattle scenarios.
% Inputs: cfg, optional defaultConfig output; each scenario runs for
% cfg.pocket.demoDurationS s with an ego scripted at cfg.perception.demoEgoSpeedMps.
% Outputs: metrics with figure handle and frame count per scenario. Position
% and clearance units are m, time s, and yaw rad.
% Example: metrics = demoPocket(defaultConfig());

if nargin < 1 || isempty(cfg)
    cfg = defaultConfig();
end
scenarioNames = {'market','cattleCrossing'};
frameCount = round(cfg.pocket.demoDurationS/cfg.pocket.demoDtS);
figureHandle = figure('Name', 'MargDarshan safe-pocket demo', ...
    'Color', 'w');
axesHandle = axes(figureHandle);
for scenarioIndex = 1:numel(scenarioNames)
    scenario = createScenario(scenarioNames{scenarioIndex}, cfg, ...
        cfg.perception.randomSeed+scenarioIndex);
    agents = scenario.agents;
    ego = scenario.egoStart;
    emptyAgents = types.agentList(cfg);
    for frame = 1:frameCount
        dt = cfg.pocket.demoDtS;
        ego.v = cfg.perception.demoEgoSpeedMps;
        [agents, ~, scenario] = stepWorld( ...
            scenario, agents, ego, dt, cfg);
        ego.x = ego.x+ego.v*cos(ego.yaw)*dt;
        ego.y = ego.y+ego.v*sin(ego.yaw)*dt;
        staticMap = groundTruthGrid(scenario, emptyAgents, ego, cfg);
        predictions = predictAgents(agents, ego, ...
            scenario.road.boundaries(1:scenario.road.boundaryCount,:), cfg);
        dynamicRisk = predictionRiskMap(predictions, ego, cfg);
        riskField = computeRiskField(staticMap, dynamicRisk, ego, cfg);
        pocketState = generateSafePocket(riskField, cfg);
        pocketCorridor(pocketState, riskField, cfg);
        plotPocket(scenario, agents, ego, pocketState, ...
            riskField, axesHandle, cfg);
        drawnow;
        pause(cfg.pocket.demoPauseS);
    end
end
metrics = struct( ...
    'figure', figureHandle, ...
    'scenarioNames', {scenarioNames}, ...
    'framesPerScenario', frameCount);
end
