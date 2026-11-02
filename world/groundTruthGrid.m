function map = groundTruthGrid(scenario, agentList, ego, cfg)
%GROUNDTRUTHGRID Rasterise oracle occupancy and surface cost in ego frame.
% Inputs: scenario and agentList, world-frame truth; ego, ego pose; cfg,
% defaultConfig output. Positions/dimensions are m, yaw rad, time s.
% Outputs: map, types.gridMap with [ny x nx] layers; grid x is forward and
% y is left relative to ego. Occupancy, traversability, and costs are [0,1].
% Example: map = groundTruthGrid(s, s.agents, s.egoStart, defaultConfig());

if nargin < 4 || isempty(cfg)
    cfg = defaultConfig();
end
map = types.gridMap(cfg);
map.originX = -((cfg.grid.nx-1)/2) * cfg.grid.resolution;
map.originY = -((cfg.grid.ny-1)/2) * cfg.grid.resolution;
localX = map.originX + (0:cfg.grid.nx-1)*cfg.grid.resolution;
localY = map.originY + (0:cfg.grid.ny-1)*cfg.grid.resolution;
[localGridX, localGridY] = meshgrid(localX, localY);
cosYaw = cos(ego.yaw);
sinYaw = sin(ego.yaw);
worldX = ego.x + cosYaw*localGridX - sinYaw*localGridY;
worldY = ego.y + sinYaw*localGridX + cosYaw*localGridY;
roadPolygon = scenario.road.drivableArea( ...
    1:scenario.road.drivableCount, :);
roadMask = inpolygon(worldX, worldY, roadPolygon(:,1), roadPolygon(:,2));
map.traversability = double(roadMask);
map.occupancy = zeros(cfg.grid.ny, cfg.grid.nx);

for k = 1:scenario.obstacles.count
    obstacle = scenario.obstacles.items(k);
    corners = rectangleCorners(obstacle.x, obstacle.y, obstacle.yaw, ...
        obstacle.length, obstacle.width);
    mask = inpolygon(worldX, worldY, corners(:,1), corners(:,2));
    map.occupancy(mask) = 1;
end
for k = 1:agentList.count
    agent = agentList.agents(k);
    if agent.valid
        corners = rectangleCorners(agent.x, agent.y, agent.yaw, ...
            agent.length, agent.width);
        mask = inpolygon(worldX, worldY, corners(:,1), corners(:,2));
        map.occupancy(mask) = 1;
    end
end
map.obstacleProb = map.occupancy;
map.traversability(map.occupancy > 0) = 0;
for k = 1:scenario.potholes.count
    pothole = scenario.potholes.items(k);
    mask = (worldX-pothole.x).^2 + (worldY-pothole.y).^2 ...
        <= pothole.radius^2;
    map.surfaceCost(mask) = max(map.surfaceCost(mask), pothole.cost);
end
map.timestamp = scenario.time;
end

function corners = rectangleCorners(x, y, yaw, lengthValue, widthValue)
localCorners = [lengthValue/2 widthValue/2; ...
    -lengthValue/2 widthValue/2; -lengthValue/2 -widthValue/2; ...
    lengthValue/2 -widthValue/2];
rotation = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
corners = localCorners * rotation' + [x y];
end
