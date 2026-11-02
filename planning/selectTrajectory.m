function [selected, selectedIndex, fallbackCode, candidates] = ...
        selectTrajectory(candidates, candidateCount, reference, egoState, ...
        pocketState, previousDecision, cfg, horizon)
%SELECTTRAJECTORY Pick the least-cost feasible path or a braking fallback.
% Inputs: candidates, fixed trajectory array; count; ego-local reference;
% egoState in world ENU; pocketState; previousDecision (optional); cfg.
% Outputs: selected trajectory, candidate index (0 for braking fallback),
% fallbackCode (0 nominal, 1 evasive, 2 comfortable brake, 3 emergency).
% Example: [path,index,fallback] = selectTrajectory(paths,n,ref,ego,p,prev,cfg);

if nargin < 7 || isempty(cfg)
    cfg = defaultConfig();
end
if nargin < 8 || isempty(horizon)
    horizon = cfg.planner.horizon;
end
if nargin < 6
    previousDecision = [];
end
capacity = cfg.planner.numCandidates;
if numel(candidates) ~= capacity || candidateCount < 0 || ...
        candidateCount > capacity || candidateCount ~= floor(candidateCount)
    error('selectTrajectory:InvalidCandidates', ...
        'Candidate array and count must match cfg.planner.numCandidates.');
end
selectedIndex = 0;
fallbackCode = 0;

previousPath = [];
if isstruct(previousDecision) && isfield(previousDecision, ...
        'selectedTrajectory')
    previousPath = previousDecision.selectedTrajectory;
end
feasibleIndices = find([candidates(1:candidateCount).feasible]);
if ~isempty(feasibleIndices)
    adjustedCosts = zeros(size(feasibleIndices));
    for k = 1:numel(feasibleIndices)
        candidateIndex = feasibleIndices(k);
        adjustedCosts(k) = candidates(candidateIndex).cost;
        if hasComparablePath(previousPath, cfg)
            distance = hypot(previousPath.x-candidates(candidateIndex).x, ...
                previousPath.y-candidates(candidateIndex).y);
            if mean(distance) <= cfg.planner.temporalConsistencyToleranceM
                candidates(candidateIndex).cost = ...
                    candidates(candidateIndex).cost ...
                    - cfg.planner.temporalConsistencyBonus;
                candidates(candidateIndex).costBreakdown.temporalConsistency ...
                    = -cfg.planner.temporalConsistencyBonus;
                adjustedCosts(k) = candidates(candidateIndex).cost;
            end
        end
    end
    [~, best] = min(adjustedCosts);
    selectedIndex = feasibleIndices(best);
    selected = candidates(selectedIndex);
    return;
end

evasiveIndices = zeros(candidateCount, 1);
evasiveCount = 0;
for k = 1:candidateCount
    if candidates(k).rejectionCode == 6 && ...
            candidates(k).costBreakdown.maxCollisionRisk <= ...
            cfg.planner.fallbackRiskThreshold
        evasiveCount = evasiveCount+1;
        evasiveIndices(evasiveCount) = k;
    end
end
if evasiveCount > 0
    evasiveIndices = evasiveIndices(1:evasiveCount);
    scores = zeros(evasiveCount, 1);
    for k = 1:evasiveCount
        path = candidates(evasiveIndices(k));
        scores(k) = path.costBreakdown.collisionRisk ...
            + path.costBreakdown.trafficRisk ...
            + path.cost*eps(max(1, abs(path.cost)));
    end
    [~, best] = min(scores);
    selectedIndex = evasiveIndices(best);
    selected = candidates(selectedIndex);
    fallbackCode = 1;
    return;
end

selected = brakingTrajectory(reference, egoState, ...
    cfg.planner.comfortableDecelMps2, horizon, cfg);
if trajectoryInPocket(selected, pocketState, egoState, cfg)
    fallbackCode = 2;
else
    selected = brakingTrajectory(reference, egoState, ...
        cfg.vehicle.maxDecel, horizon, cfg);
    fallbackCode = 3;
end
end

function yes = hasComparablePath(previousPath, cfg)
yes = isstruct(previousPath) && all(isfield(previousPath, {'x','y'})) && ...
    numel(previousPath.x) == cfg.planner.samples && ...
    numel(previousPath.y) == cfg.planner.samples && ...
    all(isfinite([previousPath.x(:); previousPath.y(:)]));
end

function path = brakingTrajectory(reference, egoState, deceleration, horizon, cfg)
path = types.trajectory(cfg);
path.t = linspace(0, horizon, cfg.planner.samples)';
acceleration = -min(deceleration, cfg.vehicle.maxDecel);
stopTime = max(0, egoState.v/(-acceleration));
activeTime = min(path.t, stopTime);
distance = max(0, egoState.v*activeTime ...
    + 0.5*acceleration*activeTime.^2);
speed = max(0, egoState.v+acceleration*path.t);
routeS = [0; cumsum(hypot(diff(reference.x(:)), diff(reference.y(:))))];
valid = [true; diff(routeS) > eps(max(1,routeS(end)))];
routeS = routeS(valid);
routeX = reference.x(:);
routeY = reference.y(:);
routeHeading = unwrap(reference.heading(:));
routeCurvature = reference.curvature(:);
routeX = routeX(valid);
routeY = routeY(valid);
routeHeading = routeHeading(valid);
routeCurvature = routeCurvature(valid);
localDistance = min(distance, routeS(end));
localX = interp1(routeS, routeX, localDistance, 'linear');
localY = interp1(routeS, routeY, localDistance, 'linear');
pastEnd = distance > routeS(end);
localX(pastEnd) = routeX(end)+(distance(pastEnd)-routeS(end))* ...
    cos(routeHeading(end));
localY(pastEnd) = routeY(end)+(distance(pastEnd)-routeS(end))* ...
    sin(routeHeading(end));
cosYaw = cos(egoState.yaw);
sinYaw = sin(egoState.yaw);
path.x = egoState.x+cosYaw*localX-sinYaw*localY;
path.y = egoState.y+sinYaw*localX+cosYaw*localY;
path.yaw = egoState.yaw+interp1(routeS, routeHeading, ...
    min(distance,routeS(end)), 'linear');
path.kappa = interp1(routeS, routeCurvature, ...
    min(distance,routeS(end)), 'linear');
path.v = speed;
path.targetSpeed = 0.0;
path.rejectionCode = 7+double(deceleration >= cfg.vehicle.maxDecel);
path.kappa(:) = 0.0;
path.cost = inf;
end

function inside = trajectoryInPocket(path, pocketState, egoState, cfg)
inside = false;
if ~isfield(pocketState, 'mask') || ...
        ~isequal(size(pocketState.mask), [cfg.grid.ny cfg.grid.nx])
    return;
end
originX = -((cfg.grid.nx-1)/2)*cfg.grid.resolution;
originY = -((cfg.grid.ny-1)/2)*cfg.grid.resolution;
localX = cos(egoState.yaw)*(path.x-egoState.x) ...
    + sin(egoState.yaw)*(path.y-egoState.y);
localY = -sin(egoState.yaw)*(path.x-egoState.x) ...
    + cos(egoState.yaw)*(path.y-egoState.y);
columns = round((localX-originX)/cfg.grid.resolution)+1;
rows = round((localY-originY)/cfg.grid.resolution)+1;
insideGrid = rows >= 1 & rows <= cfg.grid.ny & ...
    columns >= 1 & columns <= cfg.grid.nx;
if all(insideGrid)
    indices = sub2ind([cfg.grid.ny cfg.grid.nx], rows, columns);
    inside = all(pocketState.mask(indices));
end
end
