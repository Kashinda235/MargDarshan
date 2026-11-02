function [steer, diagnostics] = lateralController(egoState, trajectory, cfg)
%LATERALCONTROLLER Track a world-frame trajectory with pure pursuit or Stanley.
% Inputs: egoState position (m), yaw (rad), and speed (m/s); trajectory
% x/y (m), yaw (rad), and kappa (1/m); cfg contains controller/vehicle data.
% Outputs: steering command (rad) and tracking diagnostics.
% Example: [steer,info] = lateralController(ego,path,cfg);

if nargin < 3 || isempty(cfg)
    cfg = defaultConfig();
end
validateTrajectory(trajectory);
[nearest, crossTrack, headingError] = nearestReference(egoState, trajectory);
lookahead = min(cfg.controller.lookaheadMaxM, max( ...
    cfg.controller.lookaheadMinM, ...
    cfg.controller.lookaheadMinM+cfg.controller.lookaheadTimeS*max(0,egoState.v)));
target = nearest;
travelled = 0.0;
for k = nearest:numel(trajectory.x)-1
    travelled = travelled+hypot(trajectory.x(k+1)-trajectory.x(k), ...
        trajectory.y(k+1)-trajectory.y(k));
    target = k+1;
    if travelled >= lookahead
        break;
    end
end
targetYaw = trajectory.yaw(target);
targetCurvature = trajectory.kappa(target);
feedforwardCurvature = cfg.controller.curvatureFeedforwardGain ...
    *targetCurvature;
feedforward = atan(cfg.vehicle.wheelbase*feedforwardCurvature);

if strcmpi(cfg.controller.lateralMethod, 'purePursuit')
    dx = trajectory.x(target)-egoState.x;
    dy = trajectory.y(target)-egoState.y;
    alpha = atan2(dy,dx)-egoState.yaw;
    alpha = atan2(sin(alpha),cos(alpha));
    distance = max(hypot(dx,dy), cfg.controller.lookaheadMinM);
    pursuitCurvature = 2*sin(alpha)/distance;
    steer = atan(cfg.vehicle.wheelbase*pursuitCurvature)+feedforward;
elseif strcmpi(cfg.controller.lateralMethod, 'stanley')
    steer = cfg.controller.headingKp*headingError ...
        -atan2(cfg.controller.stanleyGain*crossTrack, ...
        max(0,egoState.v)+cfg.controller.stanleySofteningMps)+feedforward;
else
    error('lateralController:InvalidMethod', ...
        'cfg.controller.lateralMethod must be purePursuit or stanley.');
end
steer = min(cfg.vehicle.maxSteer, max(-cfg.vehicle.maxSteer, steer));
diagnostics = struct( ...
    'crossTrackError', crossTrack, ...
    'headingError', headingError, ...
    'nearestIndex', nearest, ...
    'targetIndex', target, ...
    'lookaheadM', lookahead, ...
    'curvatureFeedforwardRad', feedforward, ...
    'targetYaw', targetYaw);
end

function validateTrajectory(trajectory)
required = {'x','y','yaw','kappa'};
if ~isstruct(trajectory) || ~all(isfield(trajectory,required)) || ...
        isempty(trajectory.x) || any([numel(trajectory.y), ...
        numel(trajectory.yaw),numel(trajectory.kappa)] ~= numel(trajectory.x)) ...
        || any(~isfinite([trajectory.x(:);trajectory.y(:); ...
        trajectory.yaw(:);trajectory.kappa(:)]))
    error('lateralController:InvalidTrajectory', ...
        'Trajectory must have matching, finite x, y, yaw, and kappa arrays.');
end
end

function [index, crossTrack, headingError] = nearestReference(ego, path)
[~,index] = min((path.x(:)-ego.x).^2+(path.y(:)-ego.y).^2);
dx = ego.x-path.x(index);
dy = ego.y-path.y(index);
crossTrack = -sin(path.yaw(index))*dx+cos(path.yaw(index))*dy;
headingError = atan2(sin(path.yaw(index)-ego.yaw), ...
    cos(path.yaw(index)-ego.yaw));
end
