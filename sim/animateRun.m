function animation = animateRun(log, cfg, options)
%ANIMATERUN Render a top-down animation from a simulation log.
% Inputs: log from runSimulation; cfg, defaultConfig output; options may
% provide frameStride (samples) and videoFile (optional output file).
% Outputs: animation with figure handle and exported filename. Coordinates
% are ENU m, yaw rad, and video frame rate is derived from logged time.
% Example: animateRun(log,defaultConfig(),struct('videoFile','run.mp4'));

if nargin < 2 || isempty(cfg)
    cfg = defaultConfig();
end
if nargin < 3 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error('animateRun:InvalidOptions', 'options must be a scalar struct.');
end
frameStride = optionValue(options, 'frameStride', 1);
validateattributes(frameStride, {'numeric'}, ...
    {'scalar','integer','positive'}, mfilename, 'options.frameStride');
videoFile = optionValue(options, 'videoFile', '');
if isstring(videoFile) && isscalar(videoFile)
    videoFile = char(videoFile);
end
if ~ischar(videoFile)
    error('animateRun:InvalidVideoFile', ...
        'options.videoFile must be a character vector or scalar string.');
end
if ~all(isfield(log, {'scenarioName','seed','time','ego','agents', ...
        'trajectory','trajectoryValid','pocket','goal', ...
        'terminationReason','planner','route'}))
    error('animateRun:InvalidLog', ...
        'log does not contain the fields produced by runSimulation.');
end

projectRoot = fileparts(fileparts(mfilename('fullpath')));
resultsDirectory = fullfile(projectRoot, 'results');
if ~isempty(videoFile)
    if isempty(fileparts(videoFile))
        if ~exist(resultsDirectory, 'dir')
            mkdir(resultsDirectory);
        end
        videoFile = fullfile(resultsDirectory, videoFile);
    else
        outputDirectory = fileparts(videoFile);
        if ~exist(outputDirectory, 'dir')
            mkdir(outputDirectory);
        end
    end
end

scenario = createScenario(log.scenarioName, cfg, log.seed);
figureVisibility = optionValue(options, 'visible', 'off');
figureHandle = figure('Name', sprintf('%s | %s', ...
    log.scenarioName, log.planner), 'Visible', figureVisibility);
writer = [];
if ~isempty(videoFile)
    writer = VideoWriter(videoFile, 'MPEG-4');
    meanStep = mean(diff(log.time));
    if isempty(meanStep) || meanStep <= 0
        meanStep = cfg.simDt;
    end
    writer.FrameRate = 1/(meanStep*frameStride);
    open(writer);
    cleanupWriter = onCleanup(@() close(writer)); %#ok<NASGU>
end

frameIndices = 1:frameStride:numel(log.time);
if frameIndices(end) ~= numel(log.time)
    frameIndices(end+1) = numel(log.time);
end
ax = axes('Parent', figureHandle);
for sample = frameIndices
    truth = scenario.agents;
    for k = 1:truth.count
        truth.agents(k).x = log.agents.x(k,sample);
        truth.agents(k).y = log.agents.y(k,sample);
        truth.agents(k).yaw = log.agents.yaw(k,sample);
        truth.agents(k).vx = log.agents.vx(k,sample);
        truth.agents(k).vy = log.agents.vy(k,sample);
        truth.agents(k).length = log.agents.length(k,sample);
        truth.agents(k).width = log.agents.width(k,sample);
        truth.agents(k).type = log.agents.type(k,sample);
        truth.agents(k).valid = log.agents.valid(k,sample);
    end
    ego = types.egoState();
    ego.x = log.ego.x(sample);
    ego.y = log.ego.y(sample);
    ego.yaw = log.ego.yaw(sample);
    ego.v = log.ego.v(sample);
    plotWorld(scenario, truth, ego, ax, cfg);
    hold(ax, 'on');
    plot(ax, log.route.x, log.route.y, ':', ...
        'Color', [0.2 0.2 0.2], 'LineWidth', 1.0);
    if log.trajectoryValid(sample)
        plot(ax, log.trajectory.x(:,sample), ...
            log.trajectory.y(:,sample), 'b-', 'LineWidth', 1.8);
    end
    boundaryCount = log.pocket.boundaryCount(sample);
    if boundaryCount > 2
        localX = log.pocket.boundaryX(1:boundaryCount,sample);
        localY = log.pocket.boundaryY(1:boundaryCount,sample);
        cosYaw = cos(ego.yaw);
        sinYaw = sin(ego.yaw);
        worldX = ego.x+cosYaw*localX-sinYaw*localY;
        worldY = ego.y+sinYaw*localX+cosYaw*localY;
        patch(ax, worldX, worldY, [0.2 0.8 0.4], ...
            'FaceAlpha', 0.18, 'EdgeColor', [0.1 0.55 0.25], ...
            'LineWidth', 1.0);
    end
    title(ax, sprintf('%s | %s | t = %.2f s | %s', ...
        log.scenarioName, log.planner, log.time(sample), ...
        log.terminationReason), 'Interpreter', 'none');
    drawnow;
    if ~isempty(writer)
        writeVideo(writer, getframe(figureHandle));
    end
end
animation = struct('figure', figureHandle, 'videoFile', videoFile);
end

function value = optionValue(options, name, defaultValue)
if isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end
