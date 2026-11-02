function summary = summarizeResults(resultsFile, outputDirectory)
%SUMMARIZERESULTS Aggregate validation runs and regenerate plots and tables.
% Inputs: resultsFile (defaults to /results/simulation_results.mat) and
% outputDirectory (defaults to /results). Confidence intervals are 95%;
% values retain SI units. Outputs: scenario/planner summary table, Markdown
% summary, and PNG/PDF figures. Example: summary = summarizeResults();

projectRoot = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1 || isempty(resultsFile)
    resultsFile = fullfile(projectRoot, 'results', ...
        'simulation_results.mat');
end
if nargin < 2 || isempty(outputDirectory)
    outputDirectory = fullfile(projectRoot, 'results');
end
if isstring(resultsFile) && isscalar(resultsFile)
    resultsFile = char(resultsFile);
end
if isstring(outputDirectory) && isscalar(outputDirectory)
    outputDirectory = char(outputDirectory);
end
if ~ischar(resultsFile) || isempty(resultsFile) ...
        || ~ischar(outputDirectory) || isempty(outputDirectory)
    error('summarizeResults:InvalidPath', ...
        'resultsFile and outputDirectory must be nonempty path strings.');
end
if ~exist(resultsFile, 'file')
    error('summarizeResults:MissingResults', ...
        'Results file does not exist: %s', resultsFile);
end
loaded = load(resultsFile);
if ~isfield(loaded, 'results') || ~istable(loaded.results)
    error('summarizeResults:InvalidResults', ...
        'MAT file must contain a results table.');
end
results = loaded.results;
required = {'Scenario','Planner','Collision','Completed', ...
    'MinimumClearanceM','MinimumTTCS','MeanReplanLatencyS', ...
    'AverageSpeedMps','TravelTimeS','CurvatureVariationPerM', ...
    'RmsLateralAccelerationMps2','RmsJerkMps3','EmergencyBrakeEvents', ...
    'EmergencyModeEvents'};
if ~all(ismember(required, results.Properties.VariableNames))
    error('summarizeResults:InvalidResults', ...
        'Results table is missing required validation metrics.');
end
if ~exist(outputDirectory, 'dir')
    mkdir(outputDirectory);
end
figuresDirectory = fullfile(outputDirectory, 'figures');
if ~exist(figuresDirectory, 'dir')
    mkdir(figuresDirectory);
end

scenarios = unique(results.Scenario, 'stable');
planners = unique(results.Planner, 'stable');
rowCount = numel(scenarios)*numel(planners);
scenarioColumn = strings(rowCount,1);
plannerColumn = strings(rowCount,1);
runCount = zeros(rowCount,1);
metrics = metricDefinitions();
estimates = nan(rowCount,numel(metrics));
confidenceLow = nan(rowCount,numel(metrics));
confidenceHigh = nan(rowCount,numel(metrics));
row = 0;
for scenarioIndex = 1:numel(scenarios)
    for plannerIndex = 1:numel(planners)
        row = row+1;
        selected = results.Scenario == scenarios(scenarioIndex) ...
            & results.Planner == planners(plannerIndex);
        rows = results(selected,:);
        scenarioColumn(row) = scenarios(scenarioIndex);
        plannerColumn(row) = planners(plannerIndex);
        runCount(row) = height(rows);
        for metricIndex = 1:numel(metrics)
            definition = metrics(metricIndex);
            values = double(rows.(definition.source));
            if definition.isRate
                [estimate, low, high] = proportionCI95(values);
            else
                values = values(isfinite(values));
                if definition.isPercentile
                    estimate = linearPercentile(values, 0.05);
                    [low, high] = percentileConfidenceInterval(values, ...
                        0.05, scenarioIndex*100+plannerIndex);
                else
                    [estimate, low, high] = meanConfidenceInterval(values);
                end
            end
            estimates(row,metricIndex) = estimate;
            confidenceLow(row,metricIndex) = low;
            confidenceHigh(row,metricIndex) = high;
        end
    end
end
summary = table(scenarioColumn, plannerColumn, runCount, ...
    estimates(:,1),estimates(:,2),estimates(:,3),estimates(:,4), ...
    estimates(:,5),estimates(:,6),estimates(:,7),estimates(:,8), ...
    estimates(:,9),estimates(:,10),estimates(:,11),estimates(:,12), ...
    estimates(:,13),confidenceLow(:,1),confidenceLow(:,2), ...
    confidenceLow(:,3),confidenceLow(:,4),confidenceLow(:,5), ...
    confidenceLow(:,6),confidenceLow(:,7),confidenceLow(:,8), ...
    confidenceLow(:,9),confidenceLow(:,10),confidenceLow(:,11), ...
    confidenceLow(:,12),confidenceLow(:,13),confidenceHigh(:,1), ...
    confidenceHigh(:,2),confidenceHigh(:,3),confidenceHigh(:,4), ...
    confidenceHigh(:,5),confidenceHigh(:,6),confidenceHigh(:,7), ...
    confidenceHigh(:,8),confidenceHigh(:,9),confidenceHigh(:,10), ...
    confidenceHigh(:,11),confidenceHigh(:,12),confidenceHigh(:,13), ...
    'VariableNames', {'Scenario','Planner','RunCount', ...
    'CollisionRate','CompletionRate','MinimumClearanceMeanM', ...
    'MinimumClearanceP05M','MinTTCMeanS','CurvatureVariationMeanPerM', ...
    'RmsLateralAccelerationMeanMps2','RmsJerkMeanMps3', ...
    'AverageSpeedMeanMps','TravelTimeMeanS','MeanReplanLatencyS', ...
    'EmergencyBrakeEventsMean','EmergencyModeEventsMean', ...
    'CollisionRateCI95Low','CompletionRateCI95Low', ...
    'MinimumClearanceMeanMCI95Low','MinimumClearanceP05MCI95Low', ...
    'MinTTCMeanSCI95Low','CurvatureVariationMeanPerMCI95Low', ...
    'RmsLateralAccelerationMeanMps2CI95Low','RmsJerkMeanMps3CI95Low', ...
    'AverageSpeedMeanMpsCI95Low','TravelTimeMeanSCI95Low', ...
    'MeanReplanLatencySCI95Low','EmergencyBrakeEventsMeanCI95Low', ...
    'EmergencyModeEventsMeanCI95Low','CollisionRateCI95High', ...
    'CompletionRateCI95High','MinimumClearanceMeanMCI95High', ...
    'MinimumClearanceP05MCI95High','MinTTCMeanSCI95High', ...
    'CurvatureVariationMeanPerMCI95High', ...
    'RmsLateralAccelerationMeanMps2CI95High','RmsJerkMeanMps3CI95High', ...
    'AverageSpeedMeanMpsCI95High','TravelTimeMeanSCI95High', ...
    'MeanReplanLatencySCI95High','EmergencyBrakeEventsMeanCI95High', ...
    'EmergencyModeEventsMeanCI95High'});
save(fullfile(outputDirectory, 'summary.mat'), 'summary');
writetable(summary, fullfile(outputDirectory, 'summary.csv'));
writeMarkdownSummary(summary, fullfile(outputDirectory, 'summary.md'));

representativeRuns = struct([]);
if isfield(loaded, 'representativeRuns')
    representativeRuns = loaded.representativeRuns;
end
plotResults(results, representativeRuns, figuresDirectory);
end

function definitions = metricDefinitions()
template = struct('name','','source','','isRate',false, ...
    'isPercentile',false);
definitions = repmat(template, 13, 1);
definitions(1) = metric('CollisionRate','Collision',true,false);
definitions(2) = metric('CompletionRate','Completed',true,false);
definitions(3) = metric('MinimumClearanceMeanM','MinimumClearanceM',false,false);
definitions(4) = metric('MinimumClearanceP05M','MinimumClearanceM',false,true);
definitions(5) = metric('MinTTCMeanS','MinimumTTCS',false,false);
definitions(6) = metric('CurvatureVariationMeanPerM','CurvatureVariationPerM',false,false);
definitions(7) = metric('RmsLateralAccelerationMeanMps2', ...
    'RmsLateralAccelerationMps2',false,false);
definitions(8) = metric('RmsJerkMeanMps3','RmsJerkMps3',false,false);
definitions(9) = metric('AverageSpeedMeanMps','AverageSpeedMps',false,false);
definitions(10) = metric('TravelTimeMeanS','TravelTimeS',false,false);
definitions(11) = metric('MeanReplanLatencyS','MeanReplanLatencyS',false,false);
definitions(12) = metric('EmergencyBrakeEventsMean', ...
    'EmergencyBrakeEvents',false,false);
definitions(13) = metric('EmergencyModeEventsMean', ...
    'EmergencyModeEvents',false,false);
end

function value = metric(name, source, isRate, isPercentile)
value = struct('name',name,'source',source, ...
    'isRate',isRate,'isPercentile',isPercentile);
end

function [meanValue, low, high] = meanConfidenceInterval(values)
if isempty(values)
    meanValue = nan;
    low = nan;
    high = nan;
    return;
end
meanValue = mean(values);
if numel(values) < 2
    halfWidth = 0.0;
else
    halfWidth = 1.96*std(values)/sqrt(numel(values));
end
low = meanValue-halfWidth;
high = meanValue+halfWidth;
end

function value = linearPercentile(values, probability)
if isempty(values)
    value = nan;
    return;
end
values = sort(values(:));
position = 1+(numel(values)-1)*probability;
lower = floor(position);
upper = ceil(position);
fraction = position-lower;
value = values(lower)*(1-fraction)+values(upper)*fraction;
end

function [low, high] = percentileConfidenceInterval(values, probability, seed)
if isempty(values)
    low = nan;
    high = nan;
    return;
end
stream = RandStream('mt19937ar','Seed',seed);
replicates = 1000;
bootValues = zeros(replicates,1);
for k = 1:replicates
    sample = values(randi(stream,numel(values),numel(values),1));
    bootValues(k) = linearPercentile(sample, probability);
end
low = linearPercentile(bootValues,0.025);
high = linearPercentile(bootValues,0.975);
end

function writeMarkdownSummary(summary, filePath)
fileId = fopen(filePath,'w');
if fileId < 0
    error('summarizeResults:SummaryOpen', ...
        'Unable to open summary file for writing: %s', filePath);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId,'# Validation summary\n\n');
fprintf(fileId,['Rates are proportions; continuous metrics are means ' ...
    'across runs, except minimum clearance P05. Brackets show 95%% ' ...
    'confidence intervals. All units are SI.\n\n']);
fprintf(fileId,['| Scenario | Planner | Runs | Collision rate | Completion rate | ' ...
    'Clearance mean (m) | Clearance P05 (m) | Min TTC mean (s) | ' ...
    'Curvature variation (1/m) | RMS lateral accel (m/s^2) | ' ...
    'RMS jerk (m/s^3) | Speed (m/s) | Travel time (s) | ' ...
    'Latency (s) | Emergency brakes | Emergency modes |\n']);
fprintf(fileId,['|---|---|---:|---:|---:|---:|---:|---:|---:|---:|' ...
    '---:|---:|---:|---:|---:|---:|\n']);
for k = 1:height(summary)
    fprintf(fileId,['| %s | %s | %d | %s | %s | %s | %s | %s | ' ...
        '%s | %s | %s | %s | %s | %s | %s | %s |\n'], ...
        char(summary.Scenario(k)), char(summary.Planner(k)), ...
        summary.RunCount(k), rateCell(summary,k,'CollisionRate'), ...
        rateCell(summary,k,'CompletionRate'), ...
        metricCell(summary,k,'MinimumClearanceMeanM'), ...
        metricCell(summary,k,'MinimumClearanceP05M'), ...
        metricCell(summary,k,'MinTTCMeanS'), ...
        metricCell(summary,k,'CurvatureVariationMeanPerM'), ...
        metricCell(summary,k,'RmsLateralAccelerationMeanMps2'), ...
        metricCell(summary,k,'RmsJerkMeanMps3'), ...
        metricCell(summary,k,'AverageSpeedMeanMps'), ...
        metricCell(summary,k,'TravelTimeMeanS'), ...
        metricCell(summary,k,'MeanReplanLatencyS'), ...
        metricCell(summary,k,'EmergencyBrakeEventsMean'), ...
        metricCell(summary,k,'EmergencyModeEventsMean'));
end
end

function text = rateCell(summary,row,name)
text = sprintf('%.1f%% [%.1f, %.1f]', ...
    100*summary.(name)(row), ...
    100*summary.([name 'CI95Low'])(row), ...
    100*summary.([name 'CI95High'])(row));
end

function text = metricCell(summary,row,name)
values = [summary.(name)(row), ...
    summary.([name 'CI95Low'])(row), ...
    summary.([name 'CI95High'])(row)];
if any(isnan(values))
    text = 'n/a';
else
    text = sprintf('%.3g [%.3g, %.3g]',values);
end
end
