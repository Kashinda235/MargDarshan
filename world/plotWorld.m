function ax = plotWorld(scenario, agentList, ego, ax, cfg)
%PLOTWORLD Draw a top-down world snapshot for inspection or animation.
% Inputs: scenario, world geometry; agentList, ground-truth agents; ego,
% ego pose; ax, optional axes handle; cfg, defaultConfig output.
% Outputs: ax, axes handle. Coordinates are ENU m and yaw is rad.
% Example: plotWorld(s, s.agents, s.egoStart);

if nargin < 4 || isempty(ax)
    ax = gca;
end
if nargin < 5 || isempty(cfg)
    cfg = defaultConfig();
end
cla(ax);
hold(ax, 'on');
polygon = scenario.road.drivableArea( ...
    1:scenario.road.drivableCount, :);
patch(ax, polygon(:,1), polygon(:,2), [0.83 0.84 0.8], ...
    'EdgeColor', 'none', 'FaceAlpha', 1.0);
boundary = scenario.road.boundaries(1:scenario.road.boundaryCount, :);
plot(ax, [boundary(:,1); boundary(1,1)], ...
    [boundary(:,2); boundary(1,2)], 'k-', 'LineWidth', 1.5);
for k = 1:scenario.road.laneMarkingCount
    line = scenario.road.laneMarkings(k, :);
    plot(ax, line([1 3]), line([2 4]), '--', ...
        'Color', [0.95 0.85 0.2], 'LineWidth', 1.0);
end
for k = 1:scenario.potholes.count
    pothole = scenario.potholes.items(k);
    theta = linspace(0, 2*pi, cfg.world.maxRoadVertices);
    patch(ax, pothole.x+pothole.radius*cos(theta), ...
        pothole.y+pothole.radius*sin(theta), [0.35 0.3 0.28], ...
        'EdgeColor', 'none');
end
for k = 1:scenario.obstacles.count
    obstacle = scenario.obstacles.items(k);
    corners = rectangleCorners(obstacle.x, obstacle.y, obstacle.yaw, ...
        obstacle.length, obstacle.width);
    patch(ax, corners(:,1), corners(:,2), [0.2 0.2 0.2], ...
        'EdgeColor', 'k', 'LineWidth', 0.8);
end
for k = 1:agentList.count
    agent = agentList.agents(k);
    if agent.valid
        corners = rectangleCorners(agent.x, agent.y, agent.yaw, ...
            agent.length, agent.width);
        patch(ax, corners(:,1), corners(:,2), ...
            cfg.world.agentColors(agent.type,:), ...
            'EdgeColor', 'k', 'LineWidth', 0.6);
    end
end
egoCorners = rectangleCorners(ego.x, ego.y, ego.yaw, ...
    cfg.vehicle.length, cfg.vehicle.width);
patch(ax, egoCorners(:,1), egoCorners(:,2), [0.1 0.75 0.95], ...
    'EdgeColor', 'k', 'LineWidth', 1.2);
plot(ax, scenario.goal(1), scenario.goal(2), 'p', ...
    'MarkerSize', 11, 'MarkerFaceColor', [0.2 0.8 0.2], ...
    'MarkerEdgeColor', 'k');
axis(ax, 'equal');
grid(ax, 'on');
xlabel(ax, 'East x (m)');
ylabel(ax, 'North y (m)');
title(ax, sprintf('%s | t = %.1f s', scenario.name, scenario.time), ...
    'Interpreter', 'none');
hold(ax, 'off');
end

function corners = rectangleCorners(x, y, yaw, lengthValue, widthValue)
localCorners = [lengthValue/2 widthValue/2; ...
    -lengthValue/2 widthValue/2; -lengthValue/2 -widthValue/2; ...
    lengthValue/2 -widthValue/2];
rotation = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
corners = localCorners * rotation' + [x y];
end
