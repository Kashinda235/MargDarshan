function log = runSimulinkScenario(scenarioName, cfg, seed, options)
%RUNSIMULINKSCENARIO Run a scenario through the MargDarshan Simulink model.
% Inputs: scenarioName, a createScenario name; cfg, defaultConfig output;
% seed, nonnegative integer; options matches runSimulation options for
% planner, durationS (s), and measureTiming.
% Outputs: log in the runSimulation format, including SI states and metrics.
% Example: log = runSimulinkScenario('village',defaultConfig(),7);

if nargin < 2 || isempty(cfg)
    cfg = defaultConfig();
end
if nargin < 3 || isempty(seed)
    seed = 1;
end
if nargin < 4 || isempty(options)
    options = struct();
end
validateattributes(seed, {'numeric'}, ...
    {'scalar','real','finite','integer','nonnegative'}, ...
    mfilename, 'seed');
if ~isstruct(options) || ~isscalar(options)
    error('runSimulinkScenario:InvalidOptions', ...
        'options must be a scalar struct.');
end

planner = optionValue(options, 'planner', 'margdarshan');
if isstring(planner) && isscalar(planner)
    planner = char(planner);
end
if ~ischar(planner) || ...
        ~any(strcmpi(planner, {'margdarshan','baseline'}))
    error('runSimulinkScenario:InvalidPlanner', ...
        'options.planner must be ''margdarshan'' or ''baseline''.');
end
planner = lower(planner);
durationS = optionValue(options, 'durationS', cfg.simulation.duration);
measureTiming = optionValue(options, 'measureTiming', true);
validateattributes(durationS, {'numeric'}, ...
    {'scalar','real','finite','positive'}, mfilename, 'options.durationS');
if ~(islogical(measureTiming) && isscalar(measureTiming))
    error('runSimulinkScenario:InvalidTimingOption', ...
        'options.measureTiming must be a logical scalar.');
end

modelName = 'MargDarshan';
modelFile = fullfile(fileparts(mfilename('fullpath')), ...
    'MargDarshan.slx');
if ~isfile(modelFile)
    buildScript = fullfile(fileparts(mfilename('fullpath')), ...
        'buildMargDarshanModel.m');
    run(buildScript);
end

initModel(cfg);
evalin('base', 'clear MDRunLog');

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
load_system(modelFile);
modelCleanup = onCleanup(@() closeLoadedModel(modelName)); %#ok<NASGU>
set_param(modelName, 'StopTime', sprintf('%.17g', durationS));
runnerBlock = [modelName '/Scenario_World/PipelineRunner'];
set_param(runnerBlock, ...
    'ScenarioName', scenarioName, ...
    'Seed', sprintf('%.17g', seed), ...
    'DurationS', sprintf('%.17g', durationS), ...
    'Planner', planner, ...
    'MeasureTiming', mat2str(measureTiming));
simulationOutput = sim(modelName, ...
    'StopTime', sprintf('%.17g', durationS), ...
    'ReturnWorkspaceOutputs', 'on');
if ~isa(simulationOutput, 'Simulink.SimulationOutput')
    error('runSimulinkScenario:InvalidSimulationOutput', ...
        'Simulink did not return a SimulationOutput object.');
end
if ~evalin('base', 'exist(''MDRunLog'', ''var'')')
    error('runSimulinkScenario:MissingRunLog', ...
        'The Simulink pipeline runner did not publish its run log.');
end
log = evalin('base', 'MDRunLog');
if ~isstruct(log) || ~all(isfield(log, ...
        {'time','ego','trajectory','minimumClearanceM','minimumTTCS', ...
         'decisionMode','cost','timing'}))
    error('runSimulinkScenario:InvalidRunLog', ...
        'The Simulink pipeline runner produced an incomplete run log.');
end
end

function value = optionValue(options, name, defaultValue)
%OPTIONVALUE Return one run option or its default.
% Inputs: options struct, name; defaultValue. Outputs: selected value.
% Example: value = optionValue(struct(),'durationS',60);
if isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end

function closeLoadedModel(modelName)
%CLOSELOADEDMODEL Close the generated model without saving run parameters.
% Inputs: modelName, loaded model name. Outputs: none.
% Example: closeLoadedModel('MargDarshan');
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
end
