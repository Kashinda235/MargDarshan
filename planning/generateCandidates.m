function [candidates, candidateCount] = generateCandidates( ...
        reference, egoState, decision, cfg)
%GENERATECANDIDATES Sample fixed-capacity Frenet-like trajectories.
% Inputs: reference, ego-local x/y (m), heading (rad), and curvature
% (1/m); egoState, world pose (m/rad) and speed (m/s); decision with target
% speed (m/s) and horizon (s); cfg, defaultConfig output.
% Outputs: candidateCount trajectories in a cfg.planner.numCandidates
% array, each with cfg.planner.samples world-frame samples. Lateral motion
% uses quintic offsets and longitudinal motion uses bounded constant accel.
% Example: [paths,n] = generateCandidates(ref, ego, d, defaultConfig());

if nargin < 4 || isempty(cfg)
    cfg = defaultConfig();
end
validateReference(reference);
if ~isstruct(egoState) || ~all(isfield(egoState, {'x','y','yaw','v'})) || ...
        ~isstruct(decision) || ...
        ~all(isfield(decision, {'targetSpeed','horizon'}))
    error('generateCandidates:InvalidInput', ...
        'egoState and decision must provide the required pose, speed, target, and horizon fields.');
end
validateattributes([egoState.x egoState.y egoState.yaw egoState.v ...
    decision.targetSpeed decision.horizon], {'numeric'}, ...
    {'vector','real','finite','numel',6}, mfilename, 'egoState/decision');
if egoState.v < 0 || decision.targetSpeed < 0 || decision.horizon <= 0
    error('generateCandidates:InvalidSpeedOrHorizon', ...
        'Speeds must be nonnegative and decision.horizon must be positive.');
end
capacity = cfg.planner.numCandidates;
candidates = repmat(types.trajectory(cfg), capacity, 1);
candidateCount = min(capacity, numel(cfg.planner.lateralOffsetsM) * ...
    numel(cfg.planner.speedFractions) * numel(cfg.planner.offsetTimesS));
time = linspace(0, decision.horizon, cfg.planner.samples)';
routeS = [0; cumsum(hypot(diff(reference.x(:)), diff(reference.y(:))))];
valid = [true; diff(routeS) > eps(max(1, routeS(end)))];
routeS = routeS(valid);
routeX = reference.x(:);
routeY = reference.y(:);
routeHeading = reference.heading(:);
routeCurvature = reference.curvature(:);
routeX = routeX(valid);
routeY = routeY(valid);
routeHeading = routeHeading(valid);
routeCurvature = routeCurvature(valid);
if numel(routeS) < 2
    error('generateCandidates:DegenerateReference', ...
        'reference must contain at least two distinct points.');
end
routeHeading = unwrap(routeHeading(:));
routeCurvature = routeCurvature(:);
cosYaw = cos(egoState.yaw);
sinYaw = sin(egoState.yaw);
index = 0;
for offsetIndex = 1:numel(cfg.planner.lateralOffsetsM)
    for speedIndex = 1:numel(cfg.planner.speedFractions)
        for timeIndex = 1:numel(cfg.planner.offsetTimesS)
            if index >= candidateCount
                break;
            end
            index = index+1;
            candidate = types.trajectory(cfg);
            targetSpeed = max(0, decision.targetSpeed* ...
                cfg.planner.speedFractions(speedIndex));
            accel = (targetSpeed-egoState.v)/decision.horizon;
            accel = min(cfg.vehicle.maxAccel, ...
                max(-cfg.vehicle.maxDecel, accel));
            speed = max(0, egoState.v+accel*time);
            distance = max(0, egoState.v*time+0.5*accel*time.^2);
            [baseX, baseY, heading, curvature] = ...
                sampleReference(routeS, routeX, routeY, routeHeading, ...
                routeCurvature, distance);
            offset = cfg.planner.lateralOffsetsM(offsetIndex);
            offsetTime = min(decision.horizon, ...
                cfg.planner.offsetTimesS(timeIndex));
            fraction = min(1.0, max(0.0, time/offsetTime));
            blend = 10*fraction.^3-15*fraction.^4+6*fraction.^5;
            localX = baseX-offset.*blend.*sin(heading);
            localY = baseY+offset.*blend.*cos(heading);
            localYaw = pathHeading(localX, localY, heading);
            pathCurvature = pathCurvatureFromXY( ...
                localYaw, distance, curvature);
            candidate.t = time;
            candidate.x = egoState.x+cosYaw*localX-sinYaw*localY;
            candidate.y = egoState.y+sinYaw*localX+cosYaw*localY;
            candidate.yaw = atan2(sin(localYaw+egoState.yaw), ...
                cos(localYaw+egoState.yaw));
            candidate.v = speed;
            candidate.kappa = pathCurvature;
            candidate.lateralOffset = offset;
            candidate.targetSpeed = targetSpeed;
            candidate.timeToOffset = offsetTime;
            candidates(index) = candidate;
        end
    end
end
end

function validateReference(reference)
required = {'x','y','heading','curvature'};
if ~isstruct(reference) || ~all(isfield(reference, required))
    error('generateCandidates:InvalidReference', ...
        'reference must provide ego-local x, y, heading, and curvature arrays.');
end
count = numel(reference.x);
if count < 2 || any([numel(reference.y), numel(reference.heading), ...
        numel(reference.curvature)] ~= count) || ...
        any(~isfinite([reference.x(:); reference.y(:); ...
        reference.heading(:); reference.curvature(:)]))
    error('generateCandidates:InvalidReference', ...
        'Reference arrays must have matching lengths and finite values.');
end
end

function [x, y, heading, curvature] = sampleReference( ...
        routeS, routeX, routeY, routeHeading, routeCurvature, distance)
clamped = min(distance, routeS(end));
x = interp1(routeS, routeX, clamped, 'linear');
y = interp1(routeS, routeY, clamped, 'linear');
heading = interp1(routeS, routeHeading, clamped, 'linear');
curvature = interp1(routeS, routeCurvature, clamped, 'linear');
pastEnd = distance > routeS(end);
x(pastEnd) = routeX(end)+(distance(pastEnd)-routeS(end))*cos(routeHeading(end));
y(pastEnd) = routeY(end)+(distance(pastEnd)-routeS(end))*sin(routeHeading(end));
heading(pastEnd) = routeHeading(end);
curvature(pastEnd) = routeCurvature(end);
end

function heading = pathHeading(x, y, fallback)
heading = fallback;
if numel(x) > 1
    heading = atan2(gradient(y), gradient(x));
    stationary = hypot(gradient(x), gradient(y)) <= eps;
    heading(stationary) = fallback(stationary);
end
end

function curvature = pathCurvatureFromXY(heading, distance, fallback)
curvature = fallback;
if numel(heading) > 2
    deltaS = gradient(distance);
    valid = deltaS > eps;
    headingChange = gradient(unwrap(heading));
    curvature(valid) = headingChange(valid)./deltaS(valid);
    curvature(~isfinite(curvature)) = 0;
end
end
