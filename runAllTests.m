function results = runAllTests()
%RUNALLTESTS Run all MargDarshan unit tests.
% Inputs: none.
% Outputs: results, matlab.unittest result array.
% Example: results = runAllTests();

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot);
startup();
results = runtests(fullfile(projectRoot, 'tests'));
assertSuccess(results);
end
