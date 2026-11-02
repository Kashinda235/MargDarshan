function [decision, diagnostics] = baselineLaneFollower( ...
        egoState, reference, agentList, cfg)
%BASELINELANEFOLLOWER Build a route-following ACC baseline trajectory.
% Inputs: egoState in world ENU (m, rad, m/s); reference in ego-local
% x/y (m), heading (rad), and curvature (1/m); tracked agentList; cfg.
% Outputs: decision contract with a fixed-size world-frame trajectory and
% diagnostics including emergency-brake state. Speeds are m/s and times s.
% Example: [d,debug] = baselineLaneFollower(ego,ref,agents,defaultConfig());

if nargin < 4 || isempty(cfg)
    cfg = defaultConfig();
end
if ~isstruct(agentList) || ~all(isfield(agentList, {'count','agents'})) ...
        || agentList.count < 0 || agentList.count > cfg.maxAgents
    error('baselineLaneFollower:InvalidAgentList', ...
        'agentList must have a valid count and fixed agent array.');
end
requiredReference = {'x','y','heading','curvature'};
if ~isstruct(reference) || ~all(isfield(reference, requiredReference)) ...
        || numel(reference.x) < 2 ...
        || any([numel(reference.y), numel(reference.heading), ...
        numel(reference.curvature)] ~= numel(reference.x))
    error('baselineLaneFollower:InvalidReference', ...
        'reference must contain matching arrays with at least two points.');
end

direction = [cos(egoState.yaw) sin(egoState.yaw)];
leaderGap = inf;
leaderSpeed = cfg.baseline.targetSpeedMps;
for k = 1:agentList.count
    agent = agentList.agents(k);
    if ~agent.valid
        continue;
    end
    displacement = [agent.x-egoState.x agent.y-egoState.y];
    longitudinalGap = dot(displacement, direction);
    lateralGap = abs(direction(1)*displacement(2) ...
        - direction(2)*displacement(1));
    if longitudinalGap <= 0 || lateralGap > cfg.baseline.laneHalfWidthM
        continue;
    end
    clearGap = max(0, longitudinalGap ...
        -(cfg.vehicle.length+agent.length)/2);
    if clearGap < leaderGap
        leaderGap = clearGap;
        leaderSpeed = max(0, dot([agent.vx agent.vy], direction));
    end
end

targetSpeed = cfg.baseline.targetSpeedMps;
emergencyBrake = false;
if isfinite(leaderGap)
    closingSpeed = egoState.v-leaderSpeed;
    if closingSpeed > 0
        timeToCollision = leaderGap/closingSpeed;
    else
        timeToCollision = inf;
    end
    targetSpeed = min(targetSpeed, max(0, leaderSpeed ...
        +(leaderGap-cfg.baseline.standstillGapM) ...
        /cfg.baseline.timeGapS));
    emergencyBrake = leaderGap <= cfg.baseline.standstillGapM ...
        || timeToCollision <= cfg.baseline.emergencyTtcS;
    if emergencyBrake
        targetSpeed = 0.0;
    end
end

decision = types.decision(cfg);
decision.targetSpeed = targetSpeed;
decision.horizon = cfg.planner.horizon;
decision.replanPeriod = cfg.baseline.replanPeriodS;
decision.mode = cfg.decisionModes.cruise;
if emergencyBrake
    decision.mode = cfg.decisionModes.emergency;
end
decision.selectedTrajectory = makeTrajectory( ...
    egoState, reference, targetSpeed, emergencyBrake, cfg);
decision.selectedTrajectory.targetSpeed = targetSpeed;
diagnostics = struct( ...
    'leaderGapM', leaderGap, ...
    'leaderSpeedMps', leaderSpeed, ...
    'emergencyBrake', emergencyBrake);
end

function path = makeTrajectory(egoState, reference, targetSpeed, ...
        emergencyBrake, cfg)
path = types.trajectory(cfg);
path.t = linspace(0, cfg.planner.horizon, cfg.planner.samples)';
if emergencyBrake
    acceleration = -cfg.vehicle.maxDecel;
else
    acceleration = min(cfg.vehicle.maxAccel, max(-cfg.vehicle.maxDecel, ...
        (targetSpeed-egoState.v)/cfg.planner.horizon));
end
path.v = max(0, egoState.v+acceleration*path.t);
distance = max(0, egoState.v*path.t ...
    + 0.5*acceleration*path.t.^2);

routeX = reference.x(:);
routeY = reference.y(:);
routeHeading = unwrap(reference.heading(:));
routeCurvature = reference.curvature(:);
routeS = [0; cumsum(hypot(diff(routeX), diff(routeY)))];
keep = [true; diff(routeS) > eps(max(1, routeS(end)))];
routeX = routeX(keep);
routeY = routeY(keep);
routeHeading = routeHeading(keep);
routeCurvature = routeCurvature(keep);
routeS = routeS(keep);
sampleDistance = min(distance, routeS(end));
localX = interp1(routeS, routeX, sampleDistance, 'linear');
localY = interp1(routeS, routeY, sampleDistance, 'linear');
path.yaw = egoState.yaw+interp1( ...
    routeS, routeHeading, sampleDistance, 'linear');
path.kappa = interp1(routeS, routeCurvature, sampleDistance, 'linear');
pastEnd = distance > routeS(end);
localX(pastEnd) = routeX(end) ...
    +(distance(pastEnd)-routeS(end))*cos(routeHeading(end));
localY(pastEnd) = routeY(end) ...
    +(distance(pastEnd)-routeS(end))*sin(routeHeading(end));
cosYaw = cos(egoState.yaw);
sinYaw = sin(egoState.yaw);
path.x = egoState.x+cosYaw*localX-sinYaw*localY;
path.y = egoState.y+sinYaw*localX+cosYaw*localY;
path.yaw = atan2(sin(path.yaw), cos(path.yaw));
path.feasible = true;
end
