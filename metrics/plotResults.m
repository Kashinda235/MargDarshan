function plotResults(results, representativeRuns, outputDirectory)
%PLOTRESULTS Export planner comparisons and representative run visualizations.
% Inputs: per-run results table, paired representative run logs, output
% directory. Units: trajectory positions m, time s, pocket area m^2.
% Outputs: PNG and PDF bar/box comparisons plus trajectory, mode, and
% pocket-area figures. Example: plotResults(T,reps,'results/figures');

if nargin < 2 || isempty(representativeRuns)
    representativeRuns = struct([]);
end
if nargin < 3 || isempty(outputDirectory)
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    outputDirectory = fullfile(projectRoot,'results','figures');
end
outputDirectory = char(outputDirectory);
if isempty(outputDirectory)
    error('plotResults:InvalidOutputDirectory', ...
        'outputDirectory must be a nonempty path string.');
end
if ~istable(results) || ~all(ismember( ...
        {'Scenario','Planner','Collision','Completed', ...
        'MinimumClearanceM','AverageSpeedMps'}, ...
        results.Properties.VariableNames))
    error('plotResults:InvalidResults', ...
        'results must contain validation rows and planner metrics.');
end
if ~exist(outputDirectory,'dir')
    mkdir(outputDirectory);
end
scenarios = unique(results.Scenario,'stable');
planners = unique(results.Planner,'stable');
fig = figure('Visible','off','Color','w');
cleanup = onCleanup(@() closeIfValid(fig));
layout = tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
plotRateBars(nexttile(layout),results,scenarios,planners, ...
    'CollisionRate','Collision','Collision rate');
plotRateBars(nexttile(layout),results,scenarios,planners, ...
    'CompletionRate','Completed','Completion rate');
title(layout,'Planner outcome comparison');
exportFigure(fig,outputDirectory,'validation_rates');
clear cleanup

fig = figure('Visible','off','Color','w');
cleanup = onCleanup(@() closeIfValid(fig));
layout = tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
plotBoxComparison(nexttile(layout),results,scenarios, ...
    'MinimumClearanceM','Minimum clearance (m)');
plotBoxComparison(nexttile(layout),results,scenarios, ...
    'AverageSpeedMps','Average speed (m/s)');
title(layout,'Run-level planner distributions');
exportFigure(fig,outputDirectory,'validation_boxplots');
clear cleanup

for k = 1:numel(representativeRuns)
    representative = representativeRuns(k);
    fig = figure('Visible','off','Color','w');
    cleanup = onCleanup(@() closeIfValid(fig));
    layout = tiledlayout(fig,1,3,'TileSpacing','compact', ...
        'Padding','compact');
    plotTrajectory(nexttile(layout),representative);
    plotModes(nexttile(layout),representative);
    plotPocketArea(nexttile(layout),representative);
    title(layout,sprintf('%s, seed %d', ...
        representative.scenario,representative.seed));
    exportFigure(fig,outputDirectory, ...
        ['representative_' char(representative.scenario)]);
    clear cleanup
end
end

function plotRateBars(ax,results,scenarios,planners,fieldName,sourceName,yLabel)
values = nan(numel(scenarios),numel(planners));
low = values;
high = values;
for scenarioIndex = 1:numel(scenarios)
    for plannerIndex = 1:numel(planners)
        selected = results.Scenario == scenarios(scenarioIndex) ...
            & results.Planner == planners(plannerIndex);
        values(scenarioIndex,plannerIndex) = ...
            mean(double(results.(sourceName)(selected)));
        [~,low(scenarioIndex,plannerIndex), ...
            high(scenarioIndex,plannerIndex)] = ...
            proportionCI95(double(results.(sourceName)(selected)));
    end
end
positions = 1:numel(scenarios);
barWidth = 0.36;
hold(ax,'on');
colors = [0.10 0.45 0.75; 0.85 0.40 0.15];
for plannerIndex = 1:numel(planners)
    x = positions+(plannerIndex-(numel(planners)+1)/2)*barWidth;
    bar(ax,x,values(:,plannerIndex),barWidth, ...
        'FaceColor',colors(min(plannerIndex,size(colors,1)),:), ...
        'DisplayName',char(planners(plannerIndex)));
    errorbar(ax,x,values(:,plannerIndex), ...
        values(:,plannerIndex)-low(:,plannerIndex), ...
        high(:,plannerIndex)-values(:,plannerIndex), ...
        'k.','LineWidth',1,'HandleVisibility','off');
end
set(ax,'XTick',positions,'XTickLabel',cellstr(scenarios));
ylabel(ax,yLabel);
ylim(ax,[0 1]);
grid(ax,'on');
legend(ax,'Location','best');
title(ax,strrep(fieldName,'Rate',''));
end

function plotBoxComparison(ax,results,scenarios,fieldName,yLabel)
plannerCategory = categorical(results.Planner, ...
    {'margdarshan','baseline'});
scenarioCategory = categorical(results.Scenario,cellstr(scenarios));
values = results.(fieldName);
finite = isfinite(values);
boxchart(ax,scenarioCategory(finite),values(finite), ...
    'GroupByColor',plannerCategory(finite));
ylabel(ax,yLabel);
xlabel(ax,'Scenario');
grid(ax,'on');
legend(ax,'Location','best');
end

function plotTrajectory(ax,representative)
hold(ax,'on');
md = representative.margdarshan;
baseline = representative.baseline;
plot(ax,md.ego.x,md.ego.y,'LineWidth',1.5, ...
    'DisplayName','MargDarshan');
plot(ax,baseline.ego.x,baseline.ego.y,'--','LineWidth',1.5, ...
    'DisplayName','Baseline');
plot(ax,md.ego.x(1),md.ego.y(1),'ko','MarkerFaceColor','g', ...
    'DisplayName','Start');
plot(ax,md.goal(1),md.goal(2),'kp','MarkerFaceColor','y', ...
    'DisplayName','Goal');
axis(ax,'equal');
grid(ax,'on');
xlabel(ax,'East (m)');
ylabel(ax,'North (m)');
title(ax,'Trajectory overlay');
legend(ax,'Location','best');
end

function plotModes(ax,representative)
hold(ax,'on');
logs = {representative.margdarshan,representative.baseline};
labels = {'MargDarshan','Baseline'};
for k = 1:numel(logs)
    log = logs{k};
    stairs(ax,log.time,log.decisionMode,'LineWidth',1.4, ...
        'DisplayName',labels{k});
end
yticks(ax,[1 2 3]);
yticklabels(ax,{'Cruise','Cautious','Emergency'});
ylim(ax,[0.8 3.2]);
xlabel(ax,'Time (s)');
ylabel(ax,'Decision mode');
title(ax,'Mode timeline');
grid(ax,'on');
legend(ax,'Location','best');
end

function plotPocketArea(ax,representative)
log = representative.margdarshan;
plot(ax,log.time,log.pocket.areaM2,'LineWidth',1.5, ...
    'DisplayName','MargDarshan pocket');
xlabel(ax,'Time (s)');
ylabel(ax,'Pocket area (m^2)');
title(ax,'Safe-pocket area');
grid(ax,'on');
legend(ax,'Location','best');
end

function exportFigure(fig,outputDirectory,baseName)
exportgraphics(fig,fullfile(outputDirectory,[baseName '.png']), ...
    'Resolution',150);
exportgraphics(fig,fullfile(outputDirectory,[baseName '.pdf']), ...
    'ContentType','vector');
end

function closeIfValid(fig)
if isgraphics(fig)
    close(fig);
end
end
