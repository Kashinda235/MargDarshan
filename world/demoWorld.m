function demoWorld(cfg)
%DEMOWORLD Animate all lightweight scenarios for 10 s with a stationary ego.
% Inputs: cfg, optional defaultConfig output; default duration is 10 s.
% Outputs: none; opens/reuses a figure. Time is s, coordinates m, yaw rad.
% Example: demoWorld();

if nargin < 1 || isempty(cfg)
    cfg = defaultConfig();
end
scenarioNames = {'village','intersection','highwayMerge','market', ...
    'cattleCrossing'};
figureHandle = figure('Name', 'MargDarshan lightweight world', ...
    'Color', 'w');
axesHandle = axes(figureHandle);
for scenarioIndex = 1:numel(scenarioNames)
    scenario = createScenario(scenarioNames{scenarioIndex}, cfg, 1);
    agents = scenario.agents;
    ego = scenario.egoStart;
    frameCount = round(cfg.world.demoDuration/cfg.world.demoDt);
    for frame = 1:frameCount
        [agents, ~, scenario] = stepWorld( ...
            scenario, agents, ego, cfg.world.demoDt, cfg);
        plotWorld(scenario, agents, ego, axesHandle, cfg);
        drawnow;
        pause(cfg.world.demoPause);
    end
end
end
