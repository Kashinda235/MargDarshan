function [candidates, feasibility] = checkFeasibility( ...
        candidates, candidateCount, pocketState, egoState, cfg, ...
        enforcePocket)
%CHECKFEASIBILITY Reject dynamically invalid or out-of-pocket paths.
% Inputs: candidates, fixed trajectory array in world ENU (m/rad); count;
% pocketState, ego-local safe mask; egoState, world pose (m/rad); cfg.
% Outputs: candidates with feasible/rejectionCode set, plus fixed-size
% metrics. Rejection codes: 1 curvature/steer, 2 lateral accel, 3 accel,
% 4 jerk, 5 outside pocket. enforcePocket (optional) disables pocket
% membership rejection for component-ablation experiments.
% Example: [paths,checks] = checkFeasibility(paths,n,pocket,ego,cfg);

if nargin < 5 || isempty(cfg)
    cfg = defaultConfig();
end
if nargin < 6 || isempty(enforcePocket)
    enforcePocket = true;
end
if ~(islogical(enforcePocket) && isscalar(enforcePocket))
    error('checkFeasibility:InvalidPocketOption', ...
        'enforcePocket must be a logical scalar.');
end
if ~isstruct(pocketState) || ~isfield(pocketState, 'mask') || ...
        ~isequal(size(pocketState.mask), [cfg.grid.ny cfg.grid.nx])
    error('checkFeasibility:InvalidPocket', ...
        'pocketState.mask must match the configured grid size.');
end
capacity = cfg.planner.numCandidates;
if numel(candidates) ~= capacity || candidateCount < 0 || ...
        candidateCount > capacity || candidateCount ~= floor(candidateCount)
    error('checkFeasibility:InvalidCandidates', ...
        'Candidate array and count must match cfg.planner.numCandidates.');
end
feasibility = struct( ...
    'feasible', false(capacity, 1), ...
    'maxCurvature', zeros(capacity, 1), ...
    'maxSteer', zeros(capacity, 1), ...
    'maxLateralAccel', zeros(capacity, 1), ...
    'maxAccel', zeros(capacity, 1), ...
    'maxJerk', zeros(capacity, 1), ...
    'inPocket', false(capacity, 1), ...
    'rejectionCode', zeros(capacity, 1));
originX = -((cfg.grid.nx-1)/2)*cfg.grid.resolution;
originY = -((cfg.grid.ny-1)/2)*cfg.grid.resolution;
cosYaw = cos(egoState.yaw);
sinYaw = sin(egoState.yaw);
accelerationTolerance = 10*eps(max(1, cfg.vehicle.maxAccel));
decelerationTolerance = 10*eps(max(1, cfg.vehicle.maxDecel));
for k = 1:candidateCount
    path = candidates(k);
    if numel(path.t) ~= cfg.planner.samples || ...
            any(~isfinite([path.t(:); path.x(:); path.y(:); ...
            path.yaw(:); path.v(:); path.kappa(:)]))
        error('checkFeasibility:InvalidTrajectory', ...
            'Each active trajectory must contain finite configured samples.');
    end
    acceleration = gradient(path.v, path.t);
    jerk = gradient(acceleration, path.t);
    feasibility.maxCurvature(k) = max(abs(path.kappa));
    feasibility.maxSteer(k) = max(abs(atan( ...
        cfg.vehicle.wheelbase*path.kappa)));
    feasibility.maxLateralAccel(k) = max(path.v.^2.*abs(path.kappa));
    feasibility.maxAccel(k) = max(abs(acceleration));
    feasibility.maxJerk(k) = max(abs(jerk));
    localX = cosYaw*(path.x-egoState.x)+sinYaw*(path.y-egoState.y);
    localY = -sinYaw*(path.x-egoState.x)+cosYaw*(path.y-egoState.y);
    columns = round((localX-originX)/cfg.grid.resolution)+1;
    rows = round((localY-originY)/cfg.grid.resolution)+1;
    insideGrid = rows >= 1 & rows <= cfg.grid.ny & ...
        columns >= 1 & columns <= cfg.grid.nx;
    pathInPocket = false(size(insideGrid));
    validIndices = find(insideGrid);
    if ~isempty(validIndices)
        linearIndices = sub2ind([cfg.grid.ny cfg.grid.nx], ...
            rows(validIndices), columns(validIndices));
        pathInPocket(validIndices) = pocketState.mask(linearIndices);
    end
    feasibility.inPocket(k) = all(pathInPocket);

    rejectionCode = 0;
    if feasibility.maxSteer(k) > cfg.vehicle.maxSteer ...
            + 10*eps(max(1,cfg.vehicle.maxSteer))
        rejectionCode = 1;
    elseif feasibility.maxLateralAccel(k) > cfg.vehicle.maxLateralAccel ...
            + 10*eps(max(1,cfg.vehicle.maxLateralAccel))
        rejectionCode = 2;
    elseif any(acceleration > cfg.vehicle.maxAccel ...
            +accelerationTolerance) || ...
            any(acceleration < -cfg.vehicle.maxDecel ...
            -decelerationTolerance)
        rejectionCode = 3;
    elseif feasibility.maxJerk(k) > cfg.vehicle.maxJerk ...
            + 10*eps(max(1,cfg.vehicle.maxJerk))
        rejectionCode = 4;
    elseif enforcePocket && ~feasibility.inPocket(k)
        rejectionCode = 5;
    end
    feasibility.rejectionCode(k) = rejectionCode;
    feasibility.feasible(k) = rejectionCode == 0;
    candidates(k).rejectionCode = rejectionCode;
    candidates(k).feasible = rejectionCode == 0;
end
end
