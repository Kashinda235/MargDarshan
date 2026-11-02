function [map, state] = buildGridMap( ...
        scenario, detections, ego, state, dt, cfg)
%BUILDGRIDMAP Update an ego-centric probabilistic grid from detections.
% Inputs: scenario, ground-truth road/pothole geometry; detections, fused
% world-frame detections; ego pose in m/rad; state, types.gridMapState;
% dt in s; cfg, defaultConfig output.
% Outputs: types.gridMap with [ny x nx] probability/cost layers and updated
% explicit temporal state. Height is m, resolution m/cell, timestamp s.
% Example: [map, state] = buildGridMap(s, fused, ego, state, 0.1, cfg);

if nargin < 6 || isempty(cfg)
    cfg = defaultConfig();
end
validateattributes(dt, {'numeric'}, ...
    {'scalar','real','finite','positive'}, mfilename, 'dt');
if isempty(state)
    state = types.gridMapState(cfg);
end
if detections.count < 0 || detections.count > numel(detections.items)
    error('buildGridMap:InvalidDetectionCount', ...
        'detections.count must be within the fixed item capacity.');
end

map = types.gridMap(cfg);
map.originX = -((cfg.grid.nx-1)/2)*cfg.grid.resolution;
map.originY = -((cfg.grid.ny-1)/2)*cfg.grid.resolution;
localX = map.originX+(0:cfg.grid.nx-1)*cfg.grid.resolution;
localY = map.originY+(0:cfg.grid.ny-1)*cfg.grid.resolution;
[localGridX, localGridY] = meshgrid(localX, localY);
cosYaw = cos(ego.yaw);
sinYaw = sin(ego.yaw);
worldGridX = ego.x+cosYaw*localGridX-sinYaw*localGridY;
worldGridY = ego.y+sinYaw*localGridX+cosYaw*localGridY;

if state.initialized
    [staticOdds, dynamicOdds] = reprojectState( ...
        state, worldGridX, worldGridY, cfg);
    dynamicOdds = dynamicOdds ...
        * exp(-dt/cfg.perception.dynamicCellDecayTimeS);
else
    staticOdds = cfg.perception.occupancyPriorLogOdds ...
        * ones(cfg.grid.ny, cfg.grid.nx);
    dynamicOdds = zeros(cfg.grid.ny, cfg.grid.nx);
end

for k = 1:detections.count
    detection = detections.items(k);
    dx = worldGridX-detection.x;
    dy = worldGridY-detection.y;
    localObjectX = cos(detection.yaw)*dx+sin(detection.yaw)*dy;
    localObjectY = -sin(detection.yaw)*dx+cos(detection.yaw)*dy;
    occupied = abs(localObjectX) <= detection.length/2 ...
        & abs(localObjectY) <= detection.width/2;
    evidence = cfg.perception.occupancyHitLogOdds ...
        * detection.confidence;
    if detection.isStatic
        staticOdds(occupied) = staticOdds(occupied)+evidence;
    else
        dynamicOdds(occupied) = dynamicOdds(occupied)+evidence;
    end
    map.height(occupied) = max(map.height(occupied), ...
        cfg.perception.obstacleHeightM);
end

for k = 1:scenario.potholes.count
    pothole = scenario.potholes.items(k);
    [noiseX, state.randomState] = ...
        perceptionRandomNormal(state.randomState);
    [noiseY, state.randomState] = ...
        perceptionRandomNormal(state.randomState);
    [noiseCost, state.randomState] = ...
        perceptionRandomNormal(state.randomState);
    estimatedX = pothole.x ...
        + noiseX*cfg.perception.potholePositionNoiseStdM;
    estimatedY = pothole.y ...
        + noiseY*cfg.perception.potholePositionNoiseStdM;
    estimatedCost = min(1.0, max(0.0, pothole.cost ...
        + noiseCost*cfg.perception.potholeCostNoiseStd));
    potholeMask = (worldGridX-estimatedX).^2 ...
        + (worldGridY-estimatedY).^2 <= pothole.radius^2;
    map.surfaceCost(potholeMask) = max( ...
        map.surfaceCost(potholeMask), estimatedCost);
    depth = estimatedCost*cfg.perception.potholeDepthScaleM;
    map.height(potholeMask) = min(map.height(potholeMask), -depth);
end

limit = cfg.perception.occupancyLogOddsLimit;
staticOdds = min(limit, max(-limit, staticOdds));
dynamicOdds = min(limit, max(-limit, dynamicOdds));
totalOdds = min(limit, max(-limit, staticOdds+dynamicOdds));
map.occupancy = 1.0./(1.0+exp(-totalOdds));
map.obstacleProb = map.occupancy;
[heightGradientX, heightGradientY] = gradient( ...
    map.height, cfg.grid.resolution, cfg.grid.resolution);
slopeCost = min(1.0, hypot(heightGradientX, heightGradientY) ...
    / cfg.perception.maxSlope);
horizontalStep = [zeros(cfg.grid.ny, 1), ...
    abs(diff(map.height, 1, 2))];
verticalStep = [zeros(1, cfg.grid.nx); ...
    abs(diff(map.height, 1, 1))];
stepCost = min(1.0, max(horizontalStep, verticalStep) ...
    / cfg.perception.heightStepThresholdM);
weightSum = cfg.perception.traversabilitySlopeWeight ...
    + cfg.perception.traversabilityStepWeight ...
    + cfg.perception.traversabilitySurfaceWeight;
terrainCost = (cfg.perception.traversabilitySlopeWeight*slopeCost ...
    + cfg.perception.traversabilityStepWeight*stepCost ...
    + cfg.perception.traversabilitySurfaceWeight*map.surfaceCost) ...
    / weightSum;
map.traversability = (1.0-map.obstacleProb).*(1.0-terrainCost);
map.timestamp = scenario.time;

state.staticLogOdds = staticOdds;
state.dynamicLogOdds = dynamicOdds;
state.egoX = ego.x;
state.egoY = ego.y;
state.egoYaw = ego.yaw;
state.originX = map.originX;
state.originY = map.originY;
state.initialized = true;
end

function [staticOdds, dynamicOdds] = reprojectState( ...
        state, worldGridX, worldGridY, cfg)
dx = worldGridX-state.egoX;
dy = worldGridY-state.egoY;
previousLocalX = cos(state.egoYaw)*dx+sin(state.egoYaw)*dy;
previousLocalY = -sin(state.egoYaw)*dx+cos(state.egoYaw)*dy;
column = round((previousLocalX-state.originX)/cfg.grid.resolution)+1;
row = round((previousLocalY-state.originY)/cfg.grid.resolution)+1;
valid = column >= 1 & column <= cfg.grid.nx ...
    & row >= 1 & row <= cfg.grid.ny;
staticOdds = cfg.perception.occupancyPriorLogOdds ...
    * ones(cfg.grid.ny, cfg.grid.nx);
dynamicOdds = zeros(cfg.grid.ny, cfg.grid.nx);
currentIndices = find(valid);
previousIndices = sub2ind([cfg.grid.ny cfg.grid.nx], ...
    row(valid), column(valid));
staticOdds(currentIndices) = state.staticLogOdds(previousIndices);
dynamicOdds(currentIndices) = state.dynamicLogOdds(previousIndices);
end
