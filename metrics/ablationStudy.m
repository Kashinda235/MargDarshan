function impact = ablationStudy(options)
%ABLATIONSTUDY Measure one-at-a-time MargDarshan component ablations.
% Inputs: options.seedCount (default 30), seedStart, durationS, and
% outputDirectory. Paired randomized scenes compare Boid/CV prediction,
% adaptive/fixed configuration, pocket gating on/off, and traffic-risk
% cost on/off. Outputs: per-run MAT/CSV plus impact table/Markdown.
% SI units; example: impact = ablationStudy();

if nargin < 1 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error('ablationStudy:InvalidOptions', ...
        'options must be a scalar struct.');
end
cfg = defaultConfig();
scenarioNames = {'village','intersection','highwayMerge', ...
    'market','cattleCrossing'};
variants = {'MargDarshan','Boid prediction -> CV', ...
    'Adaptive config -> fixed','Pocket gating -> off', ...
    'Traffic-risk cost -> off'};
switches = repmat(struct('predictionMethod','', ...
    'disableAdaptiveConfig',false,'disablePocketGating',false, ...
    'disableTrafficRiskCost',false),numel(variants),1);
switches(2).predictionMethod = 'cv';
switches(3).disableAdaptiveConfig = true;
switches(4).disablePocketGating = true;
switches(5).disableTrafficRiskCost = true;
seedCount = optionValue(options,'seedCount',30);
seedStart = optionValue(options,'seedStart',1);
durationS = optionValue(options,'durationS',cfg.simulation.duration);
validateattributes(seedCount,{'numeric'}, ...
    {'scalar','integer','>=',30},mfilename,'options.seedCount');
validateattributes(seedStart,{'numeric'}, ...
    {'scalar','integer','nonnegative'},mfilename,'options.seedStart');
validateattributes(durationS,{'numeric'}, ...
    {'scalar','real','finite','positive'},mfilename,'options.durationS');
projectRoot = fileparts(fileparts(mfilename('fullpath')));
outputDirectory = optionValue(options,'outputDirectory', ...
    fullfile(projectRoot,'results'));
if isstring(outputDirectory) && isscalar(outputDirectory)
    outputDirectory = char(outputDirectory);
end
if ~ischar(outputDirectory) || isempty(outputDirectory)
    error('ablationStudy:InvalidOutputDirectory', ...
        'options.outputDirectory must be a nonempty path string.');
end
if ~exist(outputDirectory,'dir')
    mkdir(outputDirectory);
end

runCount = numel(scenarioNames)*seedCount*numel(variants);
scenarioColumn = strings(runCount,1);
variantColumn = strings(runCount,1);
seedColumn = zeros(runCount,1);
collisionColumn = false(runCount,1);
completedColumn = false(runCount,1);
clearanceColumn = nan(runCount,1);
ttcColumn = nan(runCount,1);
speedColumn = nan(runCount,1);
travelTimeColumn = nan(runCount,1);
latencyColumn = nan(runCount,1);
emergencyColumn = zeros(runCount,1);
row = 0;
for scenarioIndex = 1:numel(scenarioNames)
    scenarioName = scenarioNames{scenarioIndex};
    for seedOffset = 0:seedCount-1
        seed = seedStart+seedOffset;
        scenario = randomizeScenario( ...
            createScenario(scenarioName,cfg,seed),cfg,seed);
        for variantIndex = 1:numel(variants)
            runLog = runSimulation(scenarioName,cfg,seed,struct( ...
                'planner','margdarshan','durationS',durationS, ...
                'measureTiming',true,'initialScenario',scenario, ...
                'ablation',switches(variantIndex)));
            metrics = computeMetrics(runLog);
            row = row+1;
            scenarioColumn(row) = string(scenarioName);
            variantColumn(row) = string(variants{variantIndex});
            seedColumn(row) = seed;
            collisionColumn(row) = metrics.collision;
            completedColumn(row) = metrics.scenarioCompletion;
            clearanceColumn(row) = metrics.minimumClearanceM;
            ttcColumn(row) = metrics.minimumTTCS;
            speedColumn(row) = metrics.averageSpeedMps;
            travelTimeColumn(row) = metrics.travelTimeS;
            latencyColumn(row) = metrics.replanningLatencyMeanS;
            emergencyColumn(row) = metrics.emergencyBrakeEvents;
        end
    end
end
runs = table(scenarioColumn,variantColumn,seedColumn,collisionColumn, ...
    completedColumn,clearanceColumn,ttcColumn,speedColumn, ...
    travelTimeColumn,latencyColumn,emergencyColumn, ...
    'VariableNames',{'Scenario','Variant','Seed','Collision', ...
    'Completed','MinimumClearanceM','MinimumTTCS','AverageSpeedMps', ...
    'TravelTimeS','MeanReplanLatencyS','EmergencyBrakeEvents'});
impact = summarizeImpact(runs,scenarioNames,variants);
save(fullfile(outputDirectory,'ablation_results.mat'),'runs','impact');
writetable(runs,fullfile(outputDirectory,'ablation_runs.csv'));
writetable(impact,fullfile(outputDirectory,'ablation_summary.csv'));
writeImpactMarkdown(impact,fullfile(outputDirectory,'ablation_summary.md'));
end

function impact = summarizeImpact(runs,scenarioNames,variants)
rowCount = numel(scenarioNames)*numel(variants);
scenarioColumn = strings(rowCount,1);
variantColumn = strings(rowCount,1);
collisionRate = nan(rowCount,1);
collisionDelta = nan(rowCount,1);
completionRate = nan(rowCount,1);
completionDelta = nan(rowCount,1);
clearanceMean = nan(rowCount,1);
clearanceDelta = nan(rowCount,1);
speedMean = nan(rowCount,1);
speedDelta = nan(rowCount,1);
travelMean = nan(rowCount,1);
travelDelta = nan(rowCount,1);
latencyMean = nan(rowCount,1);
latencyDelta = nan(rowCount,1);
emergencyMean = nan(rowCount,1);
emergencyDelta = nan(rowCount,1);
row = 0;
for scenarioIndex = 1:numel(scenarioNames)
    reference = runs.Scenario == string(scenarioNames{scenarioIndex}) ...
        & runs.Variant == string(variants{1});
    for variantIndex = 1:numel(variants)
        row = row+1;
        selected = runs.Scenario == string(scenarioNames{scenarioIndex}) ...
            & runs.Variant == string(variants{variantIndex});
        scenarioColumn(row) = string(scenarioNames{scenarioIndex});
        variantColumn(row) = string(variants{variantIndex});
        collisionRate(row) = mean(double(runs.Collision(selected)));
        completionRate(row) = mean(double(runs.Completed(selected)));
        clearanceMean(row) = finiteMean(runs.MinimumClearanceM(selected));
        speedMean(row) = mean(runs.AverageSpeedMps(selected));
        travelMean(row) = mean(runs.TravelTimeS(selected));
        latencyMean(row) = mean(runs.MeanReplanLatencyS(selected));
        emergencyMean(row) = mean(runs.EmergencyBrakeEvents(selected));
        collisionDelta(row) = collisionRate(row) ...
            -mean(double(runs.Collision(reference)));
        completionDelta(row) = completionRate(row) ...
            -mean(double(runs.Completed(reference)));
        clearanceDelta(row) = clearanceMean(row) ...
            -finiteMean(runs.MinimumClearanceM(reference));
        speedDelta(row) = speedMean(row) ...
            -mean(runs.AverageSpeedMps(reference));
        travelDelta(row) = travelMean(row) ...
            -mean(runs.TravelTimeS(reference));
        latencyDelta(row) = latencyMean(row) ...
            -mean(runs.MeanReplanLatencyS(reference));
        emergencyDelta(row) = emergencyMean(row) ...
            -mean(runs.EmergencyBrakeEvents(reference));
    end
end
impact = table(scenarioColumn,variantColumn,collisionRate,collisionDelta, ...
    completionRate,completionDelta,clearanceMean,clearanceDelta, ...
    speedMean,speedDelta,travelMean,travelDelta,latencyMean,latencyDelta, ...
    emergencyMean,emergencyDelta,'VariableNames',{'Scenario','Variant', ...
    'CollisionRate','CollisionRateDelta','CompletionRate', ...
    'CompletionRateDelta','MinimumClearanceMeanM','ClearanceDeltaM', ...
    'AverageSpeedMeanMps','AverageSpeedDeltaMps','TravelTimeMeanS', ...
    'TravelTimeDeltaS','MeanReplanLatencyMeanS','LatencyDeltaS', ...
    'EmergencyBrakeEventsMean','EmergencyBrakeEventsDelta'});
end

function value = finiteMean(values)
values = values(isfinite(values));
if isempty(values)
    value = nan;
else
    value = mean(values);
end
end

function writeImpactMarkdown(impact,filePath)
fileId = fopen(filePath,'w');
if fileId < 0
    error('ablationStudy:SummaryOpen', ...
        'Unable to open ablation summary: %s',filePath);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId,'# One-at-a-time ablation study\n\n');
fprintf(fileId,['Deltas are variant minus the MargDarshan reference ' ...
    'on paired randomized runs. Positive collision delta is worse; ' ...
    'positive clearance/completion delta is better.\n\n']);
fprintf(fileId,['| Scenario | Variant | Collision rate | Delta | Completion rate | ' ...
    'Delta | Clearance mean (m) | Delta (m) | Speed (m/s) | ' ...
    'Travel time (s) | Latency (s) | Emergency brakes |\n']);
fprintf(fileId,['|---|---|---:|---:|---:|---:|---:|---:|---:|---:|' ...
    '---:|---:|\n']);
for k = 1:height(impact)
    fprintf(fileId,['| %s | %s | %.3f | %+.3f | %.3f | %+.3f | ' ...
        '%.3g | %+.3g | %.3g | %.3g | %.3g | %.3g |\n'], ...
        char(impact.Scenario(k)),char(impact.Variant(k)), ...
        impact.CollisionRate(k),impact.CollisionRateDelta(k), ...
        impact.CompletionRate(k),impact.CompletionRateDelta(k), ...
        impact.MinimumClearanceMeanM(k),impact.ClearanceDeltaM(k), ...
        impact.AverageSpeedMeanMps(k),impact.TravelTimeMeanS(k), ...
        impact.MeanReplanLatencyMeanS(k),impact.EmergencyBrakeEventsMean(k));
end
end

function value = optionValue(options,name,defaultValue)
if isfield(options,name)
    value = options.(name);
else
    value = defaultValue;
end
end
