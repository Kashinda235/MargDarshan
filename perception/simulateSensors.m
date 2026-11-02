function [frame, randomState] = simulateSensors( ...
        scenario, truth, ego, cfg, randomState)
%SIMULATESENSORS Simulate noisy camera, lidar, and radar observations.
% Inputs: scenario and truth, world-frame ground truth; ego, ENU pose and
% speed (m, rad, m/s); cfg, defaultConfig output; randomState, explicit
% nonnegative integer PRNG state (optional).
% Outputs: fixed-capacity sensor frame; detection positions/extents are
% world-frame m, yaw rad, radar radial velocity m/s, and camera bbox is
% [forward left length width] in ego-frame m. Also returns randomState.
% Example: [frame, seed] = simulateSensors(s, s.agents, s.egoStart, cfg, 17);

if nargin < 4 || isempty(cfg)
    cfg = defaultConfig();
end
if nargin < 5
    randomState = cfg.perception.randomSeed;
end
validateattributes(randomState, {'numeric'}, ...
    {'scalar','real','finite','integer','nonnegative'}, ...
    mfilename, 'randomState');
if truth.count < 0 || truth.count > cfg.maxAgents
    error('simulateSensors:InvalidAgentCount', ...
        'truth.count must be between 0 and cfg.maxAgents.');
end

capacity = cfg.perception.maxDetections;
frame = struct( ...
    'camera', emptyList(capacity), ...
    'lidar', emptyList(capacity), ...
    'radar', emptyList(capacity), ...
    'egoX', ego.x, ...
    'egoY', ego.y, ...
    'egoYaw', ego.yaw, ...
    'egoVelocityX', ego.v*cos(ego.yaw), ...
    'egoVelocityY', ego.v*sin(ego.yaw), ...
    'timestamp', scenario.time);
sources = emptySources(cfg.maxAgents + cfg.world.maxStaticObstacles);
sourceCount = 0;
for k = 1:truth.count
    if truth.agents(k).valid
        sourceCount = sourceCount + 1;
        sources(sourceCount) = makeSource(truth.agents(k), false);
    end
end
for k = 1:scenario.obstacles.count
    obstacle = scenario.obstacles.items(k);
    sourceCount = sourceCount + 1;
    source = emptySources(1);
    source.x = obstacle.x;
    source.y = obstacle.y;
    source.vx = 0.0;
    source.vy = 0.0;
    source.yaw = obstacle.yaw;
    source.length = obstacle.length;
    source.width = obstacle.width;
    source.type = obstacle.type;
    source.isStatic = true;
    sources(sourceCount) = source;
end

distance = zeros(1, sourceCount);
for k = 1:sourceCount
    distance(k) = hypot(sources(k).x-ego.x, sources(k).y-ego.y);
end
[~, order] = sort(distance);

for q = 1:sourceCount
    k = order(q);
    source = sources(k);
    range = distance(k);
    if range <= cfg.sensors.cameraRange
        probability = cfg.perception.cameraDetectionBase ...
            * exp(-range/cfg.perception.cameraDetectionDecayM);
        [draw, randomState] = perceptionRandomUniform(randomState);
        if draw <= probability
            [noiseX, randomState] = perceptionRandomNormal(randomState);
            [noiseY, randomState] = perceptionRandomNormal(randomState);
            detection = makeDetection(source);
            detection.x = source.x ...
                + noiseX*cfg.perception.cameraPositionNoiseStdM;
            detection.y = source.y ...
                + noiseY*cfg.perception.cameraPositionNoiseStdM;
            detection.confidence = probability;
            detection.positionVariance = ...
                cfg.perception.cameraPositionNoiseStdM^2;
            [confuse, randomState] = ...
                perceptionRandomUniform(randomState);
            if confuse < cfg.perception.cameraClassConfusionProb
                [classDraw, randomState] = ...
                    perceptionRandomUniform(randomState);
                guessedType = floor(classDraw*9) + 1;
                if guessedType >= source.type
                    guessedType = guessedType + 1;
                end
                detection.type = guessedType;
                detection.confidence = detection.confidence ...
                    * (1.0-cfg.perception.cameraClassConfusionProb);
            end
            local = worldToEgo(detection.x, detection.y, ego);
            relativeYaw = source.yaw-ego.yaw;
            forwardExtent = abs(cos(relativeYaw))*source.length ...
                + abs(sin(relativeYaw))*source.width;
            lateralExtent = abs(sin(relativeYaw))*source.length ...
                + abs(cos(relativeYaw))*source.width;
            [extentNoise, randomState] = ...
                perceptionRandomNormal(randomState);
            forwardExtent = max(cfg.perception.minimumDetectionExtentM, ...
                forwardExtent + extentNoise ...
                * cfg.perception.cameraExtentNoiseStdM);
            [extentNoise, randomState] = ...
                perceptionRandomNormal(randomState);
            lateralExtent = max(cfg.perception.minimumDetectionExtentM, ...
                lateralExtent + extentNoise ...
                * cfg.perception.cameraExtentNoiseStdM);
            detection.length = forwardExtent;
            detection.width = lateralExtent;
            detection.bbox = [local(1) local(2) ...
                forwardExtent lateralExtent];
            frame.camera = append(frame.camera, detection);
        end
    end

    if range <= cfg.sensors.lidarRange && ~isOccluded( ...
            source, sources, sourceCount, order, distance, ego)
        [draw, randomState] = perceptionRandomUniform(randomState);
        if draw <= cfg.perception.lidarDetectionProbability
            detection = makeDetection(source);
            [noiseX, randomState] = perceptionRandomNormal(randomState);
            [noiseY, randomState] = perceptionRandomNormal(randomState);
            [noiseLength, randomState] = ...
                perceptionRandomNormal(randomState);
            [noiseWidth, randomState] = ...
                perceptionRandomNormal(randomState);
            [noiseYaw, randomState] = perceptionRandomNormal(randomState);
            detection.x = source.x ...
                + noiseX*cfg.perception.lidarPositionNoiseStdM;
            detection.y = source.y ...
                + noiseY*cfg.perception.lidarPositionNoiseStdM;
            detection.width = max(cfg.perception.minimumDetectionExtentM, ...
                source.width ...
                + noiseWidth*cfg.perception.lidarExtentNoiseStdM);
            detection.length = max(cfg.perception.minimumDetectionExtentM, ...
                source.length ...
                + noiseLength*cfg.perception.lidarExtentNoiseStdM);
            detection.yaw = source.yaw ...
                + noiseYaw*cfg.perception.lidarYawNoiseStdRad;
            detection.type = cfg.agentTypes.car;
            detection.confidence = cfg.perception.lidarDetectionProbability;
            detection.positionVariance = ...
                cfg.perception.lidarPositionNoiseStdM^2;
            detection.isStatic = source.isStatic;
            frame.lidar = append(frame.lidar, detection);
        end
    end

    if range <= cfg.sensors.radarRange
        [draw, randomState] = perceptionRandomUniform(randomState);
        if draw <= cfg.perception.radarDetectionProbability
            detection = makeDetection(source);
            detection.length = 0.0;
            detection.width = 0.0;
            [noiseX, randomState] = perceptionRandomNormal(randomState);
            [noiseY, randomState] = perceptionRandomNormal(randomState);
            [noiseVelocity, randomState] = ...
                perceptionRandomNormal(randomState);
            detection.x = source.x ...
                + noiseX*cfg.perception.radarPositionNoiseStdM;
            detection.y = source.y ...
                + noiseY*cfg.perception.radarPositionNoiseStdM;
            direction = [source.x-ego.x source.y-ego.y] ...
                / max(range, eps);
            egoVelocity = [frame.egoVelocityX frame.egoVelocityY];
            sourceVelocity = [source.vx source.vy];
            detection.radialVelocity = dot( ...
                sourceVelocity-egoVelocity, direction) ...
                + noiseVelocity ...
                * cfg.perception.radarRadialVelocityNoiseStdMps;
            detection.confidence = cfg.perception.radarDetectionProbability;
            detection.positionVariance = ...
                cfg.perception.radarPositionNoiseStdM^2;
            detection.radialVelocityVariance = ...
                cfg.perception.radarRadialVelocityNoiseStdMps^2;
            frame.radar = append(frame.radar, detection);
        end
    end
end

[frame.camera, randomState] = addFalsePositives(frame.camera, ...
    cfg.perception.cameraFalsePositiveMean, cfg.sensors.cameraRange, ...
    cfg, randomState, ego, true, false, ...
    cfg.perception.cameraPositionNoiseStdM);
[frame.lidar, randomState] = addFalsePositives(frame.lidar, ...
    cfg.perception.lidarFalsePositiveMean, cfg.sensors.lidarRange, ...
    cfg, randomState, ego, false, false, ...
    cfg.perception.lidarPositionNoiseStdM);
[frame.radar, randomState] = addFalsePositives(frame.radar, ...
    cfg.perception.radarFalsePositiveMean, cfg.sensors.radarRange, ...
    cfg, randomState, ego, false, true, ...
    cfg.perception.radarPositionNoiseStdM);
end

function list = emptyList(capacity)
template = emptyDetection();
list = struct('count', 0, 'items', repmat(template, 1, capacity));
end

function items = emptySources(capacity)
template = struct('x', 0.0, 'y', 0.0, 'vx', 0.0, 'vy', 0.0, ...
    'yaw', 0.0, 'length', 0.0, 'width', 0.0, ...
    'type', 1, 'isStatic', false);
items = repmat(template, 1, capacity);
end

function source = makeSource(agent, isStatic)
source = struct('x', agent.x, 'y', agent.y, 'vx', agent.vx, ...
    'vy', agent.vy, 'yaw', agent.yaw, 'length', agent.length, ...
    'width', agent.width, 'type', agent.type, 'isStatic', isStatic);
end

function detection = emptyDetection()
detection = struct('x', 0.0, 'y', 0.0, 'length', 0.0, 'width', 0.0, ...
    'yaw', 0.0, 'type', 1, 'confidence', 0.0, ...
    'radialVelocity', 0.0, 'positionVariance', 1.0, ...
    'radialVelocityVariance', 1.0, 'isStatic', false, ...
    'bbox', zeros(1, 4));
end

function detection = makeDetection(source)
detection = emptyDetection();
detection.x = source.x;
detection.y = source.y;
detection.length = source.length;
detection.width = source.width;
detection.yaw = source.yaw;
detection.type = source.type;
detection.isStatic = source.isStatic;
end

function list = append(list, detection)
if list.count < numel(list.items)
    list.count = list.count + 1;
    list.items(list.count) = detection;
end
end

function local = worldToEgo(x, y, ego)
dx = x-ego.x;
dy = y-ego.y;
local = [cos(ego.yaw)*dx+sin(ego.yaw)*dy, ...
    -sin(ego.yaw)*dx+cos(ego.yaw)*dy];
end

function occluded = isOccluded(target, sources, sourceCount, ...
        order, distance, ego)
targetRange = hypot(target.x-ego.x, target.y-ego.y);
occluded = false;
if targetRange <= eps
    return
end
targetPosition = [target.x target.y];
for q = 1:sourceCount
    blockerIndex = order(q);
    if distance(blockerIndex) >= targetRange
        break
    end
    blocker = sources(blockerIndex);
    if lineIntersectsRectangle([ego.x ego.y], targetPosition, blocker)
        occluded = true;
        return
    end
end
end

function intersects = lineIntersectsRectangle(startPoint, endPoint, box)
rotation = [cos(box.yaw) sin(box.yaw); ...
    -sin(box.yaw) cos(box.yaw)];
localStart = (startPoint-[box.x box.y])*rotation';
localEnd = (endPoint-[box.x box.y])*rotation';
direction = localEnd-localStart;
lower = [-box.length/2 -box.width/2];
upper = [box.length/2 box.width/2];
tMinimum = 0.0;
tMaximum = 1.0;
intersects = true;
for axis = 1:2
    if abs(direction(axis)) <= eps
        if localStart(axis) < lower(axis) ...
                || localStart(axis) > upper(axis)
            intersects = false;
            return
        end
    else
        t1 = (lower(axis)-localStart(axis))/direction(axis);
        t2 = (upper(axis)-localStart(axis))/direction(axis);
        tNear = min(t1, t2);
        tFar = max(t1, t2);
        tMinimum = max(tMinimum, tNear);
        tMaximum = min(tMaximum, tFar);
        if tMinimum > tMaximum
            intersects = false;
            return
        end
    end
end
intersects = intersects && tMinimum < 1.0 && tMaximum > 0.0;
end

function [list, randomState] = addFalsePositives( ...
        list, meanCount, maxRange, cfg, randomState, ego, camera, ...
        radar, positionNoiseStd)
integerCount = floor(meanCount);
fraction = meanCount-integerCount;
[draw, randomState] = perceptionRandomUniform(randomState);
falseCount = integerCount + double(draw < fraction);
falseCount = min(falseCount, cfg.perception.maxFalsePositives);
for k = 1:falseCount
    [angleDraw, randomState] = perceptionRandomUniform(randomState);
    [rangeDraw, randomState] = perceptionRandomUniform(randomState);
    angle = 2*pi*angleDraw;
    range = cfg.perception.falsePositiveMinRangeM ...
        + rangeDraw*max(0, maxRange ...
        - cfg.perception.falsePositiveMinRangeM);
    detection = emptyDetection();
    localX = range*cos(angle);
    localY = range*sin(angle);
    detection.x = ego.x + cos(ego.yaw)*localX-sin(ego.yaw)*localY;
    detection.y = ego.y + sin(ego.yaw)*localX+cos(ego.yaw)*localY;
    detection.confidence = cfg.perception.falsePositiveConfidence;
    detection.positionVariance = positionNoiseStd^2;
    if camera
        detection.length = cfg.perception.falsePositiveExtentM;
        detection.width = cfg.perception.falsePositiveExtentM;
        detection.bbox = [localX localY ...
            detection.length detection.width];
        [typeDraw, randomState] = perceptionRandomUniform(randomState);
        detection.type = floor(typeDraw*10)+1;
    end
    if radar
        [velocityNoise, randomState] = ...
            perceptionRandomNormal(randomState);
        detection.radialVelocity = velocityNoise ...
            * cfg.perception.radarRadialVelocityNoiseStdMps;
        detection.radialVelocityVariance = ...
            cfg.perception.radarRadialVelocityNoiseStdMps^2;
    end
    list = append(list, detection);
end
end
