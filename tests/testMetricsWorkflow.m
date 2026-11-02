function tests = testMetricsWorkflow()
%TESTMETRICSWORKFLOW Unit tests for deterministic validation evidence tools.
tests = functiontests(localfunctions);
end

function testScenarioRandomizationIsSeededAndBounded(testCase)
cfg = defaultConfig();
scenarioNames = {'village','intersection','highwayMerge', ...
    'market','cattleCrossing'};
for k = 1:numel(scenarioNames)
    initial = createScenario(scenarioNames{k},cfg,17);
    first = randomizeScenario(initial,cfg,41);
    second = randomizeScenario(initial,cfg,41);
    verifyEqual(testCase,first,second);
    verifyLessThanOrEqual(testCase,first.agents.count,cfg.maxAgents);
    verifyLessThanOrEqual(testCase,first.obstacles.count, ...
        cfg.world.maxStaticObstacles);
    verifyLessThanOrEqual(testCase,first.potholes.count,cfg.world.maxPotholes);
    log = runSimulation(scenarioNames{k},cfg,41,struct( ...
        'planner','margdarshan','durationS',0.1, ...
        'measureTiming',false,'initialScenario',first));
    verifyEqual(testCase,log.scenarioName,scenarioNames{k});
end
end

function testSummaryRegeneratesTablesAndFigures(testCase)
testDirectory = [tempname '_metrics'];
mkdir(testDirectory);
cleanup = onCleanup(@() removeTestDirectory(testDirectory));
Scenario = repmat("village",4,1);
Planner = ["margdarshan";"margdarshan";"baseline";"baseline"];
Seed = [1;2;1;2];
Collision = logical([0;1;0;0]);
Completed = logical([1;0;1;1]);
MinimumClearanceM = [1.0;0.8;1.4;1.2];
MinimumTTCS = [2.0;1.5;2.5;2.2];
MeanReplanLatencyS = [0.01;0.02;0.015;0.016];
AverageSpeedMps = [2.0;1.5;2.5;2.4];
TravelTimeS = [20;30;18;19];
CurvatureVariationPerM = [0.1;0.2;0.05;0.06];
RmsLateralAccelerationMps2 = [0.2;0.3;0.1;0.12];
RmsJerkMps3 = [0.5;0.7;0.4;0.45];
EmergencyBrakeEvents = [1;2;0;0];
EmergencyModeEvents = [1;2;0;0];
results = table(Scenario,Planner,Seed,Collision,Completed, ...
    MinimumClearanceM,MinimumTTCS,MeanReplanLatencyS,AverageSpeedMps, ...
    TravelTimeS,CurvatureVariationPerM,RmsLateralAccelerationMps2, ...
    RmsJerkMps3,EmergencyBrakeEvents,EmergencyModeEvents);
resultsFile = fullfile(testDirectory,'source.mat');
save(resultsFile,'results');

summary = summarizeResults(resultsFile,testDirectory);
verifyEqual(testCase,height(summary),2);
verifyTrue(testCase,all(ismember( ...
    {'CollisionRate','CompletionRate','MinimumClearanceP05M', ...
    'MinTTCMeanS','RmsJerkMeanMps3'}, ...
    summary.Properties.VariableNames)));
verifyTrue(testCase,isfile(fullfile(testDirectory,'summary.md')));
verifyTrue(testCase,isfile(fullfile(testDirectory,'figures', ...
    'validation_rates.png')));
verifyTrue(testCase,isfile(fullfile(testDirectory,'figures', ...
    'validation_boxplots.pdf')));
end

function testAblationOptionsRunWithoutChangingDefaultMode(testCase)
cfg = defaultConfig();
scenario = randomizeScenario(createScenario('market',cfg,9),cfg,9);
ablation = struct('predictionMethod','cv', ...
    'disableAdaptiveConfig',true,'disablePocketGating',true, ...
    'disableTrafficRiskCost',true);
log = runSimulation('market',cfg,9,struct( ...
    'planner','margdarshan','durationS',0.1, ...
    'measureTiming',false,'initialScenario',scenario, ...
    'ablation',ablation));
verifyEqual(testCase,log.planner,'margdarshan');
verifyEqual(testCase,log.time(end),0.1,'AbsTol',cfg.simDt);
verifyEqual(testCase,log.decisionMode(2),cfg.decisionModes.cruise);
end

function testProportionConfidenceIntervalIsBounded(testCase)
[estimate,lower,upper] = proportionCI95([false;true;false;true]);
verifyEqual(testCase,estimate,0.5);
verifyGreaterThanOrEqual(testCase,lower,0);
verifyLessThanOrEqual(testCase,upper,1);
verifyLessThan(testCase,lower,estimate);
verifyGreaterThan(testCase,upper,estimate);
end

function removeTestDirectory(testDirectory)
if exist(testDirectory,'dir')
    rmdir(testDirectory,'s');
end
end
