function tests = testSimulinkParity()
%TESTSIMULINKPARITY Compare Simulink and MATLAB vehicle trajectories.
tests = functiontests(localfunctions);
end

function testVillageTrajectoryParity(testCase)
cfg = defaultConfig();
options = struct( ...
    'planner', 'margdarshan', ...
    'durationS', 2.0, ...
    'measureTiming', false);
matlabLog = runSimulation('village', cfg, 23, options);
simulinkLog = runSimulinkScenario('village', cfg, 23, options);

verifyEqual(testCase, simulinkLog.time, matlabLog.time);
positionDifferenceM = hypot( ...
    simulinkLog.ego.x-matlabLog.ego.x, ...
    simulinkLog.ego.y-matlabLog.ego.y);
verifyLessThan(testCase, max(positionDifferenceM), ...
    cfg.simulink.parityToleranceM);
verifyEqual(testCase, simulinkLog.decisionMode, matlabLog.decisionMode);
verifyEqual(testCase, simulinkLog.cost, matlabLog.cost);
verifyEqual(testCase, computeMetrics(simulinkLog), computeMetrics(matlabLog));
end
