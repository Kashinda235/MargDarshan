function metrics = computeMetrics(log)
%COMPUTEMETRICS Summarise safety, completion, timing, and motion quality.
% Inputs: log from runSimulation; distances m, speeds m/s, times s,
% accelerations m/s^2, jerk m/s^3, and curvature 1/m.
% Outputs: scalar metrics, smoothness and per-module timing structs.
% Example: summary = computeMetrics(runSimulation('village',defaultConfig()));

required = {'time','ego','collision','minimumClearanceM','minimumTTCS', ...
    'replanned','fallbackCode','emergencyBrakeEvent','timing', ...
    'completed','terminationReason'};
if ~isstruct(log) || ~all(isfield(log, required)) ...
        || isempty(log.time) || numel(log.time) ~= numel(log.ego.x) ...
        || any(~isfinite(log.time)) ...
        || any(~isfinite([log.ego.x(:); log.ego.y(:); log.ego.yaw(:); ...
        log.ego.v(:); log.ego.a(:)]))
    error('computeMetrics:InvalidLog', ...
        'log must contain finite, matching runSimulation state arrays.');
end
sampleCount = numel(log.time);
if sampleCount > 1
    dx = diff(log.ego.x(:));
    dy = diff(log.ego.y(:));
    distance = hypot(dx, dy);
    pathLengthM = sum(distance);
    travelTimeS = log.time(end)-log.time(1);
    yaw = unwrap(log.ego.yaw(:));
    yawChange = diff(yaw);
    validDistance = distance > eps;
    curvature = yawChange(validDistance)./distance(validDistance);
    speed = log.ego.v(:);
    intervalSpeed = 0.5*(speed(1:end-1)+speed(2:end));
    lateralAcceleration = intervalSpeed(validDistance).^2.*curvature;
    if isempty(curvature)
        curvatureVariation = 0.0;
        maximumLateralAcceleration = 0.0;
        rmsLateralAcceleration = 0.0;
    else
        curvatureVariation = sum(abs(diff(curvature)));
        maximumLateralAcceleration = max(abs(lateralAcceleration));
        rmsLateralAcceleration = sqrt(mean(lateralAcceleration.^2));
    end
    intervalTime = diff(log.time(:));
    validTime = intervalTime > 0;
    jerk = diff(log.ego.a(:))./intervalTime;
    jerk = jerk(validTime);
    if isempty(jerk)
        maximumJerk = 0.0;
        rmsJerk = 0.0;
    else
        maximumJerk = max(abs(jerk));
        rmsJerk = sqrt(mean(jerk.^2));
    end
    averageSpeed = mean(log.ego.v(:));
else
    pathLengthM = 0.0;
    travelTimeS = 0.0;
    curvatureVariation = 0.0;
    maximumLateralAcceleration = 0.0;
    rmsLateralAcceleration = 0.0;
    maximumJerk = 0.0;
    rmsJerk = 0.0;
    averageSpeed = log.ego.v(1);
end

finiteClearance = log.minimumClearanceM(isfinite(log.minimumClearanceM));
if isempty(finiteClearance)
    minimumClearanceM = inf;
else
    minimumClearanceM = min(finiteClearance);
end
finiteTTC = log.minimumTTCS(isfinite(log.minimumTTCS));
if isempty(finiteTTC)
    minimumTTCS = inf;
else
    minimumTTCS = min(finiteTTC);
end
replanIndices = find(log.replanned);
if isempty(replanIndices)
    latency = zeros(0,1);
else
    latency = log.timing.replanLatency(replanIndices)';
end
if isempty(latency)
    replanningLatencyMeanS = 0.0;
    replanningLatencyMaxS = 0.0;
else
    replanningLatencyMeanS = mean(latency);
    replanningLatencyMaxS = max(latency);
end

moduleNames = {'world','sensors','fusion','tracking','gridMap', ...
    'reference','prediction','planning','control','vehicle','collision'};
moduleTimingMeanS = struct();
moduleTimingMaxS = struct();
for k = 1:numel(moduleNames)
    name = moduleNames{k};
    values = log.timing.(name)(:);
    if isempty(values)
        moduleTimingMeanS.(name) = 0.0;
        moduleTimingMaxS.(name) = 0.0;
    else
        moduleTimingMeanS.(name) = mean(values);
        moduleTimingMaxS.(name) = max(values);
    end
end

metrics = struct( ...
    'collision', any(log.collision), ...
    'scenarioCompletion', logical(log.completed), ...
    'terminationReason', char(log.terminationReason), ...
    'replanningLatencyMeanS', replanningLatencyMeanS, ...
    'replanningLatencyMaxS', replanningLatencyMaxS, ...
    'moduleTimingMeanS', moduleTimingMeanS, ...
    'moduleTimingMaxS', moduleTimingMaxS, ...
    'minimumClearanceM', minimumClearanceM, ...
    'minimumTTCS', minimumTTCS, ...
    'pathSmoothness', struct( ...
        'curvatureVariationPerM', curvatureVariation, ...
        'maximumLateralAccelerationMps2', maximumLateralAcceleration, ...
        'rmsLateralAccelerationMps2', rmsLateralAcceleration, ...
        'maximumJerkMps3', maximumJerk, ...
        'rmsJerkMps3', rmsJerk), ...
    'averageSpeedMps', averageSpeed, ...
    'travelTimeS', travelTimeS, ...
    'pathLengthM', pathLengthM, ...
    'emergencyBrakeEvents', sum(log.emergencyBrakeEvent), ...
    'numberOfReplans', sum(log.replanned), ...
    'fallbackActivations', sum(log.replanned ...
        & log.fallbackCode > 0));
end
