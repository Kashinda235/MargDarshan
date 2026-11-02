function metrics = demoPlanner(cfg)
%DEMOPLANNER Compare candidate paths at frozen village/intersection snapshots.
% Inputs: cfg, optional defaultConfig output; snapshot times are in s.
% Outputs: metrics with figure, scenario names, snapshot times, selected
% modes, fallback codes, and number of snapshots. Positions are ENU m.
% Example: metrics = demoPlanner(defaultConfig());

if nargin < 1 || isempty(cfg)
    cfg = defaultConfig();
end
scenarioNames = {'village','intersection'};
snapshotTimes = cfg.planner.demoSnapshotTimesS(:)';
figureHandle = figure('Name', 'MargDarshan local planner snapshots', ...
    'Color', 'w');
selectedModes = zeros(numel(scenarioNames), numel(snapshotTimes));
fallbackCodes = zeros(size(selectedModes));
for scenarioIndex = 1:numel(scenarioNames)
    scenario = createScenario(scenarioNames{scenarioIndex}, cfg, ...
        cfg.perception.randomSeed+scenarioIndex);
    ego = scenario.egoStart;
    ego.v = cfg.perception.demoEgoSpeedMps;
    route = densifyRoute([ego.x ego.y; scenario.goal], cfg);
    [~, reference] = getLocalReference(ego, route, 1, cfg);
    previousDecision = [];
    for snapshotIndex = 1:numel(snapshotTimes)
        while scenario.time+eps < snapshotTimes(snapshotIndex)
            step = min(cfg.world.demoDt, ...
                snapshotTimes(snapshotIndex)-scenario.time);
            [scenario.agents, ~, scenario] = stepWorld( ...
                scenario, scenario.agents, ego, step, cfg);
        end
        predictions = predictAgents(scenario.agents, ego, ...
            scenario.road.boundaries(1:scenario.road.boundaryCount,:), cfg);
        gridMap = groundTruthGrid(scenario, scenario.agents, ego, cfg);
        [decision, diagnostics] = planLocal(ego, reference, gridMap, ...
            scenario.agents, predictions, previousDecision, cfg);
        selectedModes(scenarioIndex, snapshotIndex) = decision.mode;
        fallbackCodes(scenarioIndex, snapshotIndex) = decision.fallbackCode;
        previousDecision = decision;
        axesHandle = subplot(numel(scenarioNames), numel(snapshotTimes), ...
            (scenarioIndex-1)*numel(snapshotTimes)+snapshotIndex, ...
            'Parent', figureHandle);
        plotWorld(scenario, scenario.agents, ego, axesHandle, cfg);
        hold(axesHandle, 'on');
        plotCandidates(diagnostics.candidates, diagnostics.candidateCount, ...
            decision.selectedIndex, axesHandle);
        if decision.selectedIndex == 0
            hold(axesHandle, 'on');
            plot(axesHandle, decision.selectedTrajectory.x, ...
                decision.selectedTrajectory.y, 'k-', 'LineWidth', 2.8);
            hold(axesHandle, 'off');
        end
        title(axesHandle, sprintf('%s | t = %.1f s | mode %d | fallback %d', ...
            scenario.name, scenario.time, decision.mode, ...
            decision.fallbackCode), 'Interpreter', 'none');
        hold(axesHandle, 'off');
    end
end
metrics = struct( ...
    'figure', figureHandle, ...
    'scenarioNames', {scenarioNames}, ...
    'snapshotTimesS', snapshotTimes, ...
    'selectedModes', selectedModes, ...
    'fallbackCodes', fallbackCodes, ...
    'snapshotCount', numel(scenarioNames)*numel(snapshotTimes));
end
