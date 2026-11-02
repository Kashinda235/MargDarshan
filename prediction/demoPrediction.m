function metrics = demoPrediction(cfg)
%DEMOPREDICTION Replay intersection truth and compare CV with Boid forecasts.
% Inputs: cfg, optional defaultConfig output; world/prediction times are s,
% positions and displacements m, velocities m/s, and yaw rad.
% Outputs: metrics with cvADE/cvFDE/boidADE/boidFDE in m; replays ground
% truth, overlays both mean predictions, and prints the four metrics.
% Example: metrics = demoPrediction(defaultConfig());

if nargin < 1 || isempty(cfg)
    cfg = defaultConfig();
end
scenario = createScenario('intersection', cfg, 1);
ego = scenario.egoStart;
agents = scenario.agents;
frameCount = round(cfg.world.demoDuration/cfg.world.demoDt) + 1;
snapshots = repmat(types.agentList(cfg), 1, frameCount);
for frame = 1:frameCount
    snapshots(frame) = agents;
    if frame < frameCount
        [agents, ~, scenario] = stepWorld( ...
            scenario, agents, ego, cfg.world.demoDt, cfg);
    end
end

cvErrorSum = 0;
boidErrorSum = 0;
adeCount = 0;
cvFinalErrorSum = 0;
boidFinalErrorSum = 0;
fdeCount = 0;
figureHandle = figure('Name', 'MargDarshan prediction comparison', ...
    'Color', 'w');
axesHandle = axes(figureHandle);
boundary = scenario.road.boundaries(1:scenario.road.boundaryCount, :);
for frame = 1:frameCount
    currentAgents = snapshots(frame);
    cvCfg = cfg;
    cvCfg.prediction.method = 'cv';
    boidCfg = cfg;
    boidCfg.prediction.method = 'boid';
    cvPrediction = predictAgents(currentAgents, ego, boundary, cvCfg);
    boidPrediction = predictAgents(currentAgents, ego, boundary, boidCfg);
    for k = 1:cvPrediction.count
        id = cvPrediction.agents(k).id;
        if id < 1 || id > cfg.maxAgents
            continue;
        end
        for step = 1:cfg.prediction.steps
            offset = round(cvPrediction.agents(k).t(step) / ...
                cfg.world.demoDt);
            futureFrame = frame+offset;
            if futureFrame > frameCount || ...
                    ~snapshots(futureFrame).agents(id).valid
                continue;
            end
            truth = snapshots(futureFrame).agents(id);
            cvErrorSum = cvErrorSum + hypot( ...
                cvPrediction.agents(k).x(step)-truth.x, ...
                cvPrediction.agents(k).y(step)-truth.y);
            boidErrorSum = boidErrorSum + hypot( ...
                boidPrediction.agents(k).x(step)-truth.x, ...
                boidPrediction.agents(k).y(step)-truth.y);
            adeCount = adeCount + 1;
        end
        finalFrame = frame+round(cfg.prediction.horizon/ ...
            cfg.world.demoDt);
        if finalFrame <= frameCount && ...
                snapshots(finalFrame).agents(id).valid
            truth = snapshots(finalFrame).agents(id);
            finalStep = cfg.prediction.steps;
            cvFinalErrorSum = cvFinalErrorSum + hypot( ...
                cvPrediction.agents(k).x(finalStep)-truth.x, ...
                cvPrediction.agents(k).y(finalStep)-truth.y);
            boidFinalErrorSum = boidFinalErrorSum + hypot( ...
                boidPrediction.agents(k).x(finalStep)-truth.x, ...
                boidPrediction.agents(k).y(finalStep)-truth.y);
            fdeCount = fdeCount + 1;
        end
    end
    if isgraphics(figureHandle)
        frameScenario = scenario;
        frameScenario.time = (frame-1)*cfg.world.demoDt;
        plotWorld(frameScenario, currentAgents, ego, axesHandle, cfg);
        hold(axesHandle, 'on');
        plotPredictions(axesHandle, cvPrediction, [0.1 0.35 0.9], '--');
        plotPredictions(axesHandle, boidPrediction, ...
            [0.85 0.15 0.35], '-.');
        legend(axesHandle, {'CV','Boid'}, 'Location', 'best');
        drawnow;
        pause(cfg.world.demoPause);
    end
end
if adeCount == 0
    error('demoPrediction:NoADESamples', ...
        'The replay duration must include prediction samples.');
end
metrics = struct( ...
    'cvADE', cvErrorSum/adeCount, ...
    'boidADE', boidErrorSum/adeCount, ...
    'cvFDE', NaN, ...
    'boidFDE', NaN, ...
    'adeSamples', adeCount, ...
    'fdeSamples', fdeCount);
if fdeCount > 0
    metrics.cvFDE = cvFinalErrorSum/fdeCount;
    metrics.boidFDE = boidFinalErrorSum/fdeCount;
end
fprintf('Intersection prediction comparison (m):\n');
fprintf('  CV   ADE %.3f | FDE %.3f\n', metrics.cvADE, metrics.cvFDE);
fprintf('  Boid ADE %.3f | FDE %.3f\n', metrics.boidADE, metrics.boidFDE);
end

function plotPredictions(ax, predictions, color, lineStyle)
for k = 1:predictions.count
    prediction = predictions.agents(k);
    if k == 1
        plot(ax, prediction.x, prediction.y, lineStyle, ...
            'Color', color, 'LineWidth', 1.2);
    else
        plot(ax, prediction.x, prediction.y, lineStyle, ...
            'Color', color, 'LineWidth', 1.2, ...
            'HandleVisibility', 'off');
    end
end
end
