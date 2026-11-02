function fused = fuseDetections(frame, cfg)
%FUSEDETECTIONS Associate per-frame camera, lidar, and radar observations.
% Inputs: frame, simulateSensors output with world-frame positions and
% ego pose/velocity; cfg, defaultConfig output. Distances are m, velocities
% m/s, yaw rad, and confidence in [0,1].
% Outputs: fixed-capacity struct with count and fused detection items.
% Example: fused = fuseDetections(frame, defaultConfig());

if nargin < 2 || isempty(cfg)
    cfg = defaultConfig();
end
camera = frame.camera;
lidar = frame.lidar;
radar = frame.radar;
fused = emptyList(cfg.perception.maxDetections);
cameraUsed = false(1, camera.count);
radarUsed = false(1, radar.count);

for k = 1:lidar.count
    cameraIndex = nearestUnused(lidar.items(k), camera, ...
        cameraUsed, cfg.perception.associationGateM);
    radarIndex = nearestUnused(lidar.items(k), radar, ...
        radarUsed, cfg.perception.associationGateM);
    indices = [cameraIndex 0 k radarIndex];
    if cameraIndex > 0
        cameraUsed(cameraIndex) = true;
    end
    if radarIndex > 0
        radarUsed(radarIndex) = true;
    end
    fused = append(fused, combine(frame, camera, lidar, radar, indices, cfg));
end

for k = 1:camera.count
    if cameraUsed(k)
        continue
    end
    radarIndex = nearestUnused(camera.items(k), radar, ...
        radarUsed, cfg.perception.associationGateM);
    if radarIndex > 0
        radarUsed(radarIndex) = true;
    end
    fused = append(fused, combine(frame, camera, lidar, radar, ...
        [k 0 0 radarIndex], cfg));
end

for k = 1:radar.count
    if ~radarUsed(k)
        fused = append(fused, combine(frame, camera, lidar, radar, ...
            [0 0 0 k], cfg));
    end
end
end

function list = emptyList(capacity)
template = emptyFusedDetection();
list = struct('count', 0, 'items', repmat(template, 1, capacity));
end

function detection = emptyFusedDetection()
detection = struct('x', 0.0, 'y', 0.0, ...
    'length', 0.0, 'width', 0.0, 'yaw', 0.0, 'type', 1, ...
    'confidence', 0.0, 'radialVelocity', 0.0, ...
    'radialUnitX', 1.0, 'radialUnitY', 0.0, ...
    'positionVariance', 1.0, 'radialVelocityVariance', 1.0, ...
    'hasCamera', false, 'hasLidar', false, 'hasRadar', false, ...
    'isStatic', false);
end

function index = nearestUnused(query, candidates, used, gate)
index = 0;
bestDistance = gate;
for k = 1:candidates.count
    if used(k)
        continue
    end
    distance = hypot(query.x-candidates.items(k).x, ...
        query.y-candidates.items(k).y);
    if distance <= bestDistance
        bestDistance = distance;
        index = k;
    end
end
end

function result = combine(frame, camera, lidar, radar, indices, cfg)
result = emptyFusedDetection();
cameraIndex = indices(1);
lidarIndex = indices(3);
radarIndex = indices(4);
measurements = zeros(3, 2);
variances = zeros(3, 1);
count = 0;
if cameraIndex > 0
    observation = camera.items(cameraIndex);
    count = count+1;
    measurements(count,:) = [observation.x observation.y];
    variances(count) = max(observation.positionVariance, eps);
    result.hasCamera = true;
    result.type = observation.type;
    result.length = observation.bbox(3);
    result.width = observation.bbox(4);
    result.yaw = frame.egoYaw;
end
if lidarIndex > 0
    observation = lidar.items(lidarIndex);
    count = count+1;
    measurements(count,:) = [observation.x observation.y];
    variances(count) = max(observation.positionVariance, eps);
    result.hasLidar = true;
    result.length = observation.length;
    result.width = observation.width;
    result.yaw = observation.yaw;
    result.isStatic = observation.isStatic;
end
if radarIndex > 0
    observation = radar.items(radarIndex);
    count = count+1;
    measurements(count,:) = [observation.x observation.y];
    variances(count) = max(observation.positionVariance, eps);
    result.hasRadar = true;
    result.radialVelocity = observation.radialVelocity;
    result.radialVelocityVariance = observation.radialVelocityVariance;
end

precision = 1./variances(1:count);
result.positionVariance = 1/sum(precision);
weights = precision/sum(precision);
result.x = sum(weights.*measurements(1:count,1));
result.y = sum(weights.*measurements(1:count,2));
if result.hasRadar
    radial = [result.x-frame.egoX result.y-frame.egoY];
    radialNorm = hypot(radial(1), radial(2));
    if radialNorm > eps
        result.radialUnitX = radial(1)/radialNorm;
        result.radialUnitY = radial(2)/radialNorm;
    end
    egoRadialVelocity = frame.egoVelocityX*result.radialUnitX ...
        + frame.egoVelocityY*result.radialUnitY;
    result.radialVelocity = result.radialVelocity+egoRadialVelocity;
end

failureProbability = 1.0;
if cameraIndex > 0
    failureProbability = failureProbability ...
        *(1-camera.items(cameraIndex).confidence);
end
if lidarIndex > 0
    failureProbability = failureProbability ...
        *(1-lidar.items(lidarIndex).confidence);
end
if radarIndex > 0
    failureProbability = failureProbability ...
        *(1-radar.items(radarIndex).confidence);
end
result.confidence = min(1.0, 1.0-failureProbability);
result.length = max(result.length, ...
    cfg.perception.minimumDetectionExtentM);
result.width = max(result.width, ...
    cfg.perception.minimumDetectionExtentM);
end

function list = append(list, item)
if list.count < numel(list.items)
    list.count = list.count+1;
    list.items(list.count) = item;
end
end
