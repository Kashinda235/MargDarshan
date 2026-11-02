function results = runBatch(scenarioNames, seedCount, cfg, options)
%RUNBATCH Evaluate planners across scenarios and deterministic seeds.
% Inputs: scenarioNames, names accepted by createScenario; seedCount,
% positive integer N; cfg, defaultConfig output; options.planners selects
% planner names and options.durationS optionally overrides timeout.
% Outputs: MATLAB table with one row per scenario/seed/planner run; saves
% simulation_results.mat and .csv under /results by default. SI units.
% Example: T = runBatch({'village'},2,defaultConfig());

if nargin < 1 || isempty(scenarioNames)
    scenarioNames = {'village','intersection','highwayMerge', ...
        'market','cattleCrossing'};
end
if nargin < 2 || isempty(seedCount)
    seedCount = 1;
end
if nargin < 3 || isempty(cfg)
    cfg = defaultConfig();
end
if nargin < 4 || isempty(options)
    options = struct();
end
validateattributes(seedCount, {'numeric'}, ...
    {'scalar','integer','positive'}, mfilename, 'seedCount');
if ~isstruct(options) || ~isscalar(options)
    error('runBatch:InvalidOptions', 'options must be a scalar struct.');
end
scenarioNames = normalizeNames(scenarioNames, 'scenarioNames');
planners = normalizeNames(optionValue(options, 'planners', ...
    {'margdarshan','baseline'}), 'options.planners');
for k = 1:numel(planners)
    planners{k} = lower(planners{k});
    if ~any(strcmp(planners{k}, {'margdarshan','baseline'}))
        error('runBatch:InvalidPlanner', ...
            'Each planner must be ''margdarshan'' or ''baseline''.');
    end
end
seedStart = optionValue(options, 'seedStart', 1);
validateattributes(seedStart, {'numeric'}, ...
    {'scalar','integer','nonnegative'}, mfilename, 'options.seedStart');
saveResults = optionValue(options, 'saveResults', true);
if ~(islogical(saveResults) && isscalar(saveResults))
    error('runBatch:InvalidSaveOption', ...
        'options.saveResults must be a logical scalar.');
end

runCount = numel(scenarioNames)*numel(planners)*seedCount;
scenarioColumn = strings(runCount,1);
plannerColumn = strings(runCount,1);
seedColumn = zeros(runCount,1);
collisionColumn = false(runCount,1);
completedColumn = false(runCount,1);
reasonColumn = strings(runCount,1);
minimumClearanceColumn = inf(runCount,1);
minimumTTCColumn = inf(runCount,1);
meanLatencyColumn = zeros(runCount,1);
maxLatencyColumn = zeros(runCount,1);
averageSpeedColumn = zeros(runCount,1);
travelTimeColumn = zeros(runCount,1);
pathLengthColumn = zeros(runCount,1);
curvatureVariationColumn = zeros(runCount,1);
maxLatAccelColumn = zeros(runCount,1);
rmsLatAccelColumn = zeros(runCount,1);
maxJerkColumn = zeros(runCount,1);
rmsJerkColumn = zeros(runCount,1);
emergencyBrakeColumn = zeros(runCount,1);
replanColumn = zeros(runCount,1);
fallbackColumn = zeros(runCount,1);
row = 0;
simulationOptions = struct();
simulationOptions.measureTiming = optionValue(options, 'measureTiming', true);
if isfield(options, 'durationS')
    simulationOptions.durationS = options.durationS;
end
for scenarioIndex = 1:numel(scenarioNames)
    for seedOffset = 0:seedCount-1
        seed = seedStart+seedOffset;
        for plannerIndex = 1:numel(planners)
            row = row+1;
            simulationOptions.planner = planners{plannerIndex};
            runLog = runSimulation(scenarioNames{scenarioIndex}, ...
                cfg, seed, simulationOptions);
            summary = computeMetrics(runLog);
            scenarioColumn(row) = string(scenarioNames{scenarioIndex});
            plannerColumn(row) = string(planners{plannerIndex});
            seedColumn(row) = seed;
            collisionColumn(row) = summary.collision;
            completedColumn(row) = summary.scenarioCompletion;
            reasonColumn(row) = string(summary.terminationReason);
            minimumClearanceColumn(row) = summary.minimumClearanceM;
            minimumTTCColumn(row) = summary.minimumTTCS;
            meanLatencyColumn(row) = summary.replanningLatencyMeanS;
            maxLatencyColumn(row) = summary.replanningLatencyMaxS;
            averageSpeedColumn(row) = summary.averageSpeedMps;
            travelTimeColumn(row) = summary.travelTimeS;
            pathLengthColumn(row) = summary.pathLengthM;
            curvatureVariationColumn(row) = ...
                summary.pathSmoothness.curvatureVariationPerM;
            maxLatAccelColumn(row) = ...
                summary.pathSmoothness.maximumLateralAccelerationMps2;
            rmsLatAccelColumn(row) = ...
                summary.pathSmoothness.rmsLateralAccelerationMps2;
            maxJerkColumn(row) = summary.pathSmoothness.maximumJerkMps3;
            rmsJerkColumn(row) = summary.pathSmoothness.rmsJerkMps3;
            emergencyBrakeColumn(row) = summary.emergencyBrakeEvents;
            replanColumn(row) = summary.numberOfReplans;
            fallbackColumn(row) = summary.fallbackActivations;
        end
    end
end

results = table(scenarioColumn, plannerColumn, seedColumn, ...
    collisionColumn, completedColumn, reasonColumn, ...
    minimumClearanceColumn, minimumTTCColumn, meanLatencyColumn, ...
    maxLatencyColumn, averageSpeedColumn, travelTimeColumn, ...
    pathLengthColumn, curvatureVariationColumn, maxLatAccelColumn, ...
    rmsLatAccelColumn, maxJerkColumn, rmsJerkColumn, ...
    emergencyBrakeColumn, replanColumn, fallbackColumn, ...
    'VariableNames', {'Scenario','Planner','Seed','Collision', ...
    'Completed','TerminationReason','MinimumClearanceM','MinimumTTCS', ...
    'MeanReplanLatencyS','MaxReplanLatencyS','AverageSpeedMps', ...
    'TravelTimeS','PathLengthM','CurvatureVariationPerM', ...
    'MaxLateralAccelerationMps2','RmsLateralAccelerationMps2', ...
    'MaxJerkMps3','RmsJerkMps3','EmergencyBrakeEvents','Replans', ...
    'FallbackActivations'});

if saveResults
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    resultsDirectory = optionValue(options, 'outputDirectory', ...
        fullfile(projectRoot, 'results'));
    if ~ischar(resultsDirectory) && ...
            ~(isstring(resultsDirectory) && isscalar(resultsDirectory))
        error('runBatch:InvalidOutputDirectory', ...
            'options.outputDirectory must be a path string.');
    end
    resultsDirectory = char(resultsDirectory);
    if ~exist(resultsDirectory, 'dir')
        mkdir(resultsDirectory);
    end
    save(fullfile(resultsDirectory, 'simulation_results.mat'), 'results');
    writetable(results, fullfile(resultsDirectory, 'simulation_results.csv'));
end
end

function names = normalizeNames(values, argumentName)
if ischar(values)
    names = {values};
elseif isstring(values)
    names = cellstr(values(:))';
elseif iscellstr(values)
    names = reshape(values,1,[]);
else
    error('runBatch:InvalidNames', ...
        '%s must be a character vector, string array, or cell array of character vectors.', ...
        argumentName);
end
if isempty(names) || any(cellfun(@isempty, names))
    error('runBatch:InvalidNames', '%s must not be empty.', argumentName);
end
end

function value = optionValue(options, name, defaultValue)
if isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end
