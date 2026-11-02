function tests = testSimulation()
%TESTSIMULATION Integration tests for the closed-loop simulation and metrics.
tests = functiontests(localfunctions);
end

function testVillageClosedLoopLogAndDeterminism(testCase)
cfg = defaultConfig();
options = struct('planner','margdarshan', ...
    'durationS',5.0,'measureTiming',false);
first = runSimulation('village',cfg,23,options);
second = runSimulation('village',cfg,23,options);

verifyTrue(testCase,all(isfield(first, {'ego','agents','trajectory', ...
    'pocket','collision','minimumClearanceM','minimumTTCS','timing', ...
    'terminationReason','completed'})));
verifyEqual(testCase,first.planner,'margdarshan');
verifyEqual(testCase,first.time(end),5.0,'AbsTol',cfg.simDt);
verifyFalse(testCase,containsNaN(first));
verifyEqual(testCase,first,second);

summary = computeMetrics(first);
verifyTrue(testCase,islogical(summary.collision));
verifyTrue(testCase,islogical(summary.scenarioCompletion));
verifyTrue(testCase,isfinite(summary.pathLengthM));
verifyGreaterThanOrEqual(testCase,summary.numberOfReplans,1);
end

function testBaselineAndBatchUseSharedSimulationContract(testCase)
cfg = defaultConfig();
options = struct('planner','baseline', ...
    'durationS',0.5,'measureTiming',false);
baselineLog = runSimulation('village',cfg,5,options);
verifyEqual(testCase,baselineLog.planner,'baseline');
verifyTrue(testCase,all(baselineLog.trajectoryValid(2:end)));

batchOptions = struct('planners',{{'baseline'}}, ...
    'durationS',0.1,'measureTiming',false,'saveResults',false);
results = runBatch({'village'},1,cfg,batchOptions);
verifyEqual(testCase,height(results),1);
verifyEqual(testCase,results.Planner,"baseline");
verifyEqual(testCase,results.Scenario,"village");
end

function testAnimationRendersLoggedRun(testCase)
cfg = defaultConfig();
runLog = runSimulation('village',cfg,3,struct( ...
    'planner','baseline','durationS',0.1,'measureTiming',false));
animation = animateRun(runLog,cfg,struct('visible','off'));
cleanup = onCleanup(@() closeIfValid(animation.figure)); %#ok<NASGU>
verifyTrue(testCase,isgraphics(animation.figure));
verifyEmpty(testCase,animation.videoFile);
end

function found = containsNaN(value)
found = false;
if isstruct(value)
    names = fieldnames(value);
    for k = 1:numel(names)
        found = found || containsNaN(value.(names{k}));
    end
elseif isnumeric(value)
    found = any(isnan(value(:)));
end
end

function closeIfValid(handle)
if isgraphics(handle)
    close(handle);
end
end
