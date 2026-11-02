classdef MargDarshanRunner < matlab.System
    %MARGDARSHANRUNNER Replay one verified MATLAB pipeline run as a bus.
    % The pipeline is executed once in MATLAB during block setup. The block
    % then emits fixed-size contract samples at cfg.simDt.

    properties (Nontunable)
        Config = struct()
        ScenarioName = 'village'
        Seed = 1
        DurationS = 60
        Planner = 'margdarshan'
        MeasureTiming = false
    end

    properties (Access = private)
        SampleIndex = 1
        RunLog
    end

    methods (Access = protected)
        function setupImpl(obj)
            options = struct( ...
                'planner', obj.Planner, ...
                'durationS', obj.DurationS, ...
                'measureTiming', obj.MeasureTiming);
            obj.RunLog = runSimulation( ...
                obj.ScenarioName, obj.Config, obj.Seed, options);
            obj.SampleIndex = 1;
            assignin('base', 'MDRunLog', obj.RunLog);
        end

        function sample = stepImpl(obj)
            sampleIndex = min(obj.SampleIndex, numel(obj.RunLog.time));
            sample = makeSample(obj.RunLog, sampleIndex, obj.Config);
            obj.SampleIndex = obj.SampleIndex+1;
        end

        function resetImpl(obj)
            obj.SampleIndex = 1;
        end

        function sampleTime = getSampleTimeImpl(obj)
            sampleTime = createSampleTime(obj, ...
                'Type', 'Discrete', 'SampleTime', obj.Config.simDt);
        end

        function outputSize = getOutputSizeImpl(~)
            outputSize = [1 1];
        end

        function outputDataType = getOutputDataTypeImpl(~)
            outputDataType = 'MargDarshanPipelineBus';
        end

        function outputComplexity = isOutputComplexImpl(~)
            outputComplexity = false;
        end

        function outputFixedSize = isOutputFixedSizeImpl(~)
            outputFixedSize = true;
        end
    end
end

function sample = makeSample(log, index, cfg)
%MAKESAMPLE Map one runSimulation record to the fixed-size pipeline bus.
% Inputs: log, runSimulation output; index, sample index; cfg, configuration.
% Outputs: sample with positions (m), yaw (rad), speed (m/s), and bus data.
% Example: sample = makeSample(runLog,1,defaultConfig());
sample = struct();

sample.ego = types.egoState();
sample.ego.x = log.ego.x(index);
sample.ego.y = log.ego.y(index);
sample.ego.yaw = log.ego.yaw(index);
sample.ego.v = log.ego.v(index);
sample.ego.a = log.ego.a(index);
sample.ego.steer = log.ego.steer(index);
sample.ego.yawRate = log.ego.yawRate(index);

sample.agents = types.agentList(cfg);
for k = 1:cfg.maxAgents
    if ~log.agents.valid(k, index)
        continue;
    end
    agent = types.agentState(cfg);
    agent.id = k;
    agent.type = log.agents.type(k, index);
    agent.x = log.agents.x(k, index);
    agent.y = log.agents.y(k, index);
    agent.yaw = log.agents.yaw(k, index);
    agent.vx = log.agents.vx(k, index);
    agent.vy = log.agents.vy(k, index);
    agent.length = log.agents.length(k, index);
    agent.width = log.agents.width(k, index);
    agent.confidence = 1.0;
    agent.valid = true;
    sample.agents.count = sample.agents.count+1;
    sample.agents.agents(k) = agent;
end

sample.prediction = types.predictedAgent(cfg);
sample.decision = types.decision(cfg);
sample.decision.mode = log.decisionMode(index);
sample.decision.fallbackCode = log.fallbackCode(index);
sample.decision.selectedTrajectory.x = log.trajectory.x(:, index);
sample.decision.selectedTrajectory.y = log.trajectory.y(:, index);
sample.decision.selectedTrajectory.yaw = log.trajectory.yaw(:, index);
sample.decision.selectedTrajectory.v = log.trajectory.v(:, index);
sample.decision.selectedTrajectory.t = linspace( ...
    0, cfg.planner.horizon, cfg.planner.samples)';
sample.decision.selectedTrajectory.cost = log.cost(index);
sample.decision.selectedTrajectory.feasible = ...
    log.trajectoryValid(index);
sample.decision.targetSpeed = log.trajectory.v(end, index);

sample.pocket = types.pocket(cfg);
sample.pocket.boundaryCount = log.pocket.boundaryCount(index);
sample.pocket.area = log.pocket.areaM2(index);
sample.pocket.forwardDepth = log.pocket.forwardDepthM(index);
boundaryCount = sample.pocket.boundaryCount;
if boundaryCount > 0
    sample.pocket.boundary(1:boundaryCount, 1) = ...
        log.pocket.boundaryX(1:boundaryCount, index);
    sample.pocket.boundary(1:boundaryCount, 2) = ...
        log.pocket.boundaryY(1:boundaryCount, index);
end

sample.command = struct( ...
    'accel', log.command.accel(index), ...
    'steer', log.command.steer(index), ...
    'brake', log.command.brake(index));
sample.speed = log.ego.v(index);
sample.minClearance = log.minimumClearanceM(index);
sample.TTC = log.minimumTTCS(index);
sample.mode = log.decisionMode(index);
sample.cost = log.cost(index);
end
