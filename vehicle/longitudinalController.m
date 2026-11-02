function [accelCommand, controllerState, diagnostics] = ...
        longitudinalController(egoState, trajectory, controllerState, cfg)
%LONGITUDINALCONTROLLER Track planned speed using PID, feed-forward, and limits.
% Inputs: egoState speed (m/s); trajectory t (s), v (m/s), and optional
% planned acceleration a (m/s^2); controllerState explicit PID memory or [];
% cfg contains gains, acceleration bounds, jerk limit, and cfg.simDt (s).
% Outputs: acceleration command (m/s^2), updated controllerState, and
% optional normalized throttle/brake commands.
% Example: [a,s] = longitudinalController(ego,path,[],cfg);

if nargin < 4 || isempty(cfg)
    cfg = defaultConfig();
end
validateTrajectory(trajectory);
if nargin < 3 || isempty(controllerState)
    controllerState = struct('speedIntegral',0.0, ...
        'previousAccelCommand',0.0,'previousSpeedError',0.0, ...
        'initialized',false);
end
requiredState = {'speedIntegral','previousAccelCommand', ...
    'previousSpeedError','initialized'};
if ~isstruct(controllerState) || ~all(isfield(controllerState,requiredState))
    error('longitudinalController:InvalidControllerState', ...
        'controllerState must contain the longitudinal controller memory fields.');
end
if ~isfield(egoState,'v') || ~isfinite(egoState.v) || egoState.v < 0
    error('longitudinalController:InvalidEgoState', ...
        'egoState.v must be a finite nonnegative speed (m/s).');
end

nearest = nearestIndex(egoState, trajectory);
target = nearest;
targetTime = trajectory.t(nearest)+cfg.controller.speedPreviewS;
if isfield(trajectory,'t') && numel(trajectory.t) == numel(trajectory.v)
    ahead = find(trajectory.t(:) >= targetTime,1,'first');
    if ~isempty(ahead)
        target = max(nearest,ahead);
    end
end
targetSpeed = max(0,trajectory.v(target));
speedError = targetSpeed-egoState.v;
feedforward = plannedAcceleration(trajectory,target);
dt = cfg.simDt;
if controllerState.initialized
    errorRate = (speedError-controllerState.previousSpeedError)/dt;
else
    errorRate = 0.0;
end
integralCandidate = min(cfg.controller.speedIntegralLimitMps2, max( ...
    -cfg.controller.speedIntegralLimitMps2, ...
    controllerState.speedIntegral+speedError*dt));
unconstrained = feedforward+cfg.controller.speedKp*speedError ...
    +cfg.controller.speedKi*integralCandidate ...
    +cfg.controller.speedKd*errorRate;
bounded = min(cfg.vehicle.maxAccel, max(-cfg.vehicle.maxDecel,unconstrained));
if bounded == unconstrained || ...
        (bounded >= cfg.vehicle.maxAccel && speedError < 0) || ...
        (bounded <= -cfg.vehicle.maxDecel && speedError > 0)
    controllerState.speedIntegral = integralCandidate;
end
unconstrained = feedforward+cfg.controller.speedKp*speedError ...
    +cfg.controller.speedKi*controllerState.speedIntegral ...
    +cfg.controller.speedKd*errorRate;
bounded = min(cfg.vehicle.maxAccel, max(-cfg.vehicle.maxDecel,unconstrained));
maxDelta = cfg.vehicle.maxJerk*dt;
accelCommand = min(controllerState.previousAccelCommand+maxDelta, ...
    max(controllerState.previousAccelCommand-maxDelta,bounded));
accelCommand = min(cfg.vehicle.maxAccel, max(-cfg.vehicle.maxDecel, ...
    accelCommand));
controllerState.previousAccelCommand = accelCommand;
controllerState.previousSpeedError = speedError;
controllerState.initialized = true;
diagnostics = struct( ...
    'targetIndex',target, ...
    'targetSpeed',targetSpeed, ...
    'speedError',speedError, ...
    'plannedAcceleration',feedforward, ...
    'throttle',max(0,accelCommand)/cfg.vehicle.maxAccel, ...
    'brake',max(0,-accelCommand)/cfg.vehicle.maxDecel);
end

function index = nearestIndex(ego, path)
index = 1;
if all(isfield(ego,{'x','y'})) && all(isfield(path,{'x','y'}))
    [~,index] = min((path.x(:)-ego.x).^2+(path.y(:)-ego.y).^2);
end
end

function acceleration = plannedAcceleration(path,index)
if isfield(path,'a') && numel(path.a) == numel(path.v) && ...
        isfinite(path.a(index))
    acceleration = path.a(index);
elseif isfield(path,'t') && numel(path.t) == numel(path.v) ...
        && numel(path.v) > 1
    if index < numel(path.v)
        pair = index;
    else
        pair = index-1;
    end
    dt = path.t(pair+1)-path.t(pair);
    if dt > 0
        acceleration = (path.v(pair+1)-path.v(pair))/dt;
    else
        acceleration = 0.0;
    end
else
    acceleration = 0.0;
end
end

function validateTrajectory(path)
required = {'v','t'};
if ~isstruct(path) || ~all(isfield(path,required)) || isempty(path.v) || ...
        numel(path.t) ~= numel(path.v) || ...
        any(~isfinite([path.t(:);path.v(:)])) || any(path.v(:) < 0)
    error('longitudinalController:InvalidTrajectory', ...
        'Trajectory must have matching finite t (s) and nonnegative v (m/s).');
end
end
