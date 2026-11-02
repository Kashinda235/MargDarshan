function results = runValidationSuite(options)
%RUNVALIDATIONSUITE Run paired randomized validation across five scenarios.
% Inputs: options.seedCount (default 30, minimum 30), seedStart, durationS,
% outputDirectory. Each seed uses the same randomized scenario for both
% planners. Distances are m, speed m/s, times s, accelerations m/s^2.
% Outputs: per-run table; MAT/CSV and representative logs under /results.
% Example: results = runValidationSuite();

if nargin < 1 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error('runValidationSuite:InvalidOptions', ...
        'options must be a scalar struct.');
end
cfg = defaultConfig();
scenarioNames = {'village','intersection','highwayMerge', ...
    'market','cattleCrossing'};
planners = {'margdarshan','baseline'};
seedCount = optionValue(options, 'seedCount', 30);
seedStart = optionValue(options, 'seedStart', 1);
durationS = optionValue(options, 'durationS', cfg.simulation.duration);
validateattributes(seedCount, {'numeric'}, ...
    {'scalar','integer','>=',30}, mfilename, 'options.seedCount');
validateattributes(seedStart, {'numeric'}, ...
    {'scalar','integer','nonnegative'}, mfilename, 'options.seedStart');
validateattributes(durationS, {'numeric'}, ...
    {'scalar','real','finite','positive'}, mfilename, 'options.durationS');
projectRoot = fileparts(fileparts(mfilename('fullpath')));
outputDirectory = optionValue(options, 'outputDirectory', ...
    fullfile(projectRoot, 'results'));
if isstring(outputDirectory) && isscalar(outputDirectory)
    outputDirectory = char(outputDirectory);
end
if ~ischar(outputDirectory) || isempty(outputDirectory)
    error('runValidationSuite:InvalidOutputDirectory', ...
        'options.outputDirectory must be a nonempty path string.');
end
if ~exist(outputDirectory, 'dir')
    mkdir(outputDirectory);
end

runCount = numel(scenarioNames)*seedCount*numel(planners);
scenarioColumn = strings(runCount,1);
plannerColumn = strings(runCount,1);
seedColumn = zeros(runCount,1);
collisionColumn = false(runCount,1);
completedColumn = false(runCount,1);
reasonColumn = strings(runCount,1);
minimumClearanceColumn = nan(runCount,1);
minimumTTCColumn = nan(runCount,1);
meanLatencyColumn = zeros(runCount,1);
averageSpeedColumn = zeros(runCount,1);
travelTimeColumn = zeros(runCount,1);
curvatureVariationColumn = zeros(runCount,1);
rmsLateralAccelerationColumn = zeros(runCount,1);
rmsJerkColumn = zeros(runCount,1);
emergencyBrakeColumn = zeros(runCount,1);
emergencyModeColumn = zeros(runCount,1);
representativeRuns = struct( ...
    'scenario', {}, 'seed', {}, 'margdarshan', {}, 'baseline', {});
row = 0;
for scenarioIndex = 1:numel(scenarioNames)
    scenarioName = scenarioNames{scenarioIndex};
    for seedOffset = 0:seedCount-1
        seed = seedStart+seedOffset;
        scenario = randomizeScenario( ...
            createScenario(scenarioName, cfg, seed), cfg, seed);
        margdarshanLog = struct();
        baselineLog = struct();
        for plannerIndex = 1:numel(planners)
            planner = planners{plannerIndex};
            runLog = runSimulation(scenarioName, cfg, seed, struct( ...
                'planner', planner, ...
                'durationS', durationS, ...
                'measureTiming', true, ...
                'initialScenario', scenario));
            metrics = computeMetrics(runLog);
            row = row+1;
            scenarioColumn(row) = string(scenarioName);
            plannerColumn(row) = string(planner);
            seedColumn(row) = seed;
            collisionColumn(row) = metrics.collision;
            completedColumn(row) = metrics.scenarioCompletion;
            reasonColumn(row) = string(metrics.terminationReason);
            minimumClearanceColumn(row) = metrics.minimumClearanceM;
            minimumTTCColumn(row) = metrics.minimumTTCS;
            meanLatencyColumn(row) = metrics.replanningLatencyMeanS;
            averageSpeedColumn(row) = metrics.averageSpeedMps;
            travelTimeColumn(row) = metrics.travelTimeS;
            curvatureVariationColumn(row) = ...
                metrics.pathSmoothness.curvatureVariationPerM;
            rmsLateralAccelerationColumn(row) = ...
                metrics.pathSmoothness.rmsLateralAccelerationMps2;
            rmsJerkColumn(row) = metrics.pathSmoothness.rmsJerkMps3;
            emergencyBrakeColumn(row) = metrics.emergencyBrakeEvents;
            mode = runLog.decisionMode == cfg.decisionModes.emergency;
            emergencyModeColumn(row) = sum(mode ...
                & [true ~mode(1:end-1)]);
            if strcmp(planner, 'margdarshan')
                margdarshanLog = runLog;
            else
                baselineLog = runLog;
            end
        end
        if seedOffset == 0
            representativeRuns(end+1) = struct( ...
                'scenario', scenarioName, ...
                'seed', seed, ...
                'margdarshan', margdarshanLog, ...
                'baseline', baselineLog); %#ok<AGROW>
        end
    end
end

results = table(scenarioColumn, plannerColumn, seedColumn, ...
    collisionColumn, completedColumn, reasonColumn, ...
    minimumClearanceColumn, minimumTTCColumn, meanLatencyColumn, ...
    averageSpeedColumn, travelTimeColumn, curvatureVariationColumn, ...
    rmsLateralAccelerationColumn, rmsJerkColumn, emergencyBrakeColumn, ...
    emergencyModeColumn, 'VariableNames', {'Scenario','Planner','Seed', ...
    'Collision','Completed','TerminationReason','MinimumClearanceM', ...
    'MinimumTTCS','MeanReplanLatencyS','AverageSpeedMps','TravelTimeS', ...
    'CurvatureVariationPerM','RmsLateralAccelerationMps2','RmsJerkMps3', ...
    'EmergencyBrakeEvents','EmergencyModeEvents'});
save(fullfile(outputDirectory, 'simulation_results.mat'), ...
    'results','representativeRuns','scenarioNames','seedStart', ...
    'seedCount','durationS');
writetable(results, fullfile(outputDirectory, 'simulation_results.csv'));
writeFailureAnalysis(results, outputDirectory);
end

function writeFailureAnalysis(results, ~)
failed = results.Collision | ~results.Completed;
if ~any(failed)
    return;
end
projectRoot = fileparts(fileparts(mfilename('fullpath')));
docsDirectory = fullfile(projectRoot, 'docs');
if ~exist(docsDirectory, 'dir')
    mkdir(docsDirectory);
end
filePath = fullfile(docsDirectory, 'FAILURE_ANALYSIS.md');
fileId = fopen(filePath, 'w');
if fileId < 0
    error('runValidationSuite:FailureReportOpen', ...
        'Unable to open failure report for writing: %s', filePath);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '# Validation failure analysis\n\n');
fprintf(fileId, ['A run is listed if it collided or did not complete. ' ...
    'Replay commands recreate the randomized initial scenario.\n\n']);
fprintf(fileId, '| Seed | Scenario | Planner | Outcome | Likely cause | Replay command |\n');
fprintf(fileId, '|---:|---|---|---|---|---|\n');
for k = find(failed(:))'
    scenario = char(results.Scenario(k));
    planner = char(results.Planner(k));
    reason = char(results.TerminationReason(k));
    cause = likelyCause(reason, results.Collision(k));
    replay = sprintf(['matlab -batch "addpath(pwd); startup; cfg=defaultConfig(); ' ...
        'scenario=randomizeScenario(createScenario(''%s'',cfg,%d),cfg,%d); ' ...
        'log=runSimulation(''%s'',cfg,%d,struct(''planner'',''%s'', ' ...
        '''initialScenario'',scenario));"'], ...
        scenario, results.Seed(k), results.Seed(k), ...
        scenario, results.Seed(k), planner);
    fprintf(fileId, '| %d | %s | %s | %s | %s | `%s` |\n', ...
        results.Seed(k), scenario, planner, reason, cause, replay);
end
end

function cause = likelyCause(reason, collision)
if collision
    cause = 'Collision checker detected overlapping ego/agent or obstacle geometry.';
elseif strcmp(reason, 'stuck')
    cause = 'Ego remained below the configured motion threshold; inspect blocking traffic or route feasibility.';
elseif strcmp(reason, 'timeout')
    cause = 'Goal was not reached before the simulation duration elapsed.';
else
    cause = ['Run terminated with reason "' reason '". Inspect the replay log.'];
end
end

function value = optionValue(options, name, defaultValue)
if isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end
