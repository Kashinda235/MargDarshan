function [collision, minClearance, timeToCollision] = checkCollision( ...
        ego, agentList, staticObstacles, cfg, horizon)
%CHECKCOLLISION Check oriented footprints and constant-velocity TTC.
% Inputs: ego pose/speed; agentList and staticObstacles, fixed truth lists;
% cfg, defaultConfig output; horizon, optional TTC horizon in s. Dimensions
% and clearance are m, velocities m/s, yaw rad.
% Outputs: collision, minClearance, and timeToCollision column vectors, in
% order of valid agents followed by static obstacles; TTC is Inf if absent.
% Example: [hit,gap,ttc] = checkCollision(ego,agents,s.obstacles,cfg);

if nargin < 4 || isempty(cfg)
    cfg = defaultConfig();
end
if nargin < 5 || isempty(horizon)
    horizon = cfg.world.ttcHorizon;
end
validateattributes(horizon, {'numeric'}, ...
    {'scalar','real','finite','nonnegative'}, mfilename, 'horizon');
agentIndices = find([agentList.agents(1:agentList.count).valid]);
agentCount = numel(agentIndices);
obstacleCount = staticObstacles.count;
resultCount = agentCount + obstacleCount;
collision = false(resultCount, 1);
minClearance = zeros(resultCount, 1);
timeToCollision = inf(resultCount, 1);
egoRect = struct('x', ego.x, 'y', ego.y, 'yaw', ego.yaw, ...
    'length', cfg.vehicle.length, 'width', cfg.vehicle.width);
egoVelocity = ego.v * [cos(ego.yaw) sin(ego.yaw)];
outIndex = 0;
for k = 1:agentCount
    agent = agentList.agents(agentIndices(k));
    target = struct('x', agent.x, 'y', agent.y, 'yaw', agent.yaw, ...
        'length', agent.length, 'width', agent.width);
    outIndex = outIndex + 1;
    [collision(outIndex), minClearance(outIndex)] = ...
        rectangleRelation(egoRect, target);
    timeToCollision(outIndex) = sweptCollisionTime(egoRect, target, ...
        egoVelocity, [agent.vx agent.vy], horizon, collision(outIndex));
end
for k = 1:obstacleCount
    obstacle = staticObstacles.items(k);
    target = struct('x', obstacle.x, 'y', obstacle.y, ...
        'yaw', obstacle.yaw, 'length', obstacle.length, ...
        'width', obstacle.width);
    outIndex = outIndex + 1;
    [collision(outIndex), minClearance(outIndex)] = ...
        rectangleRelation(egoRect, target);
    timeToCollision(outIndex) = sweptCollisionTime(egoRect, target, ...
        egoVelocity, [0 0], horizon, collision(outIndex));
end
end

function [overlap, clearance] = rectangleRelation(first, second)
firstCorners = rectangleCorners(first);
secondCorners = rectangleCorners(second);
axes = [cos(first.yaw) sin(first.yaw); ...
    -sin(first.yaw) cos(first.yaw); cos(second.yaw) sin(second.yaw); ...
    -sin(second.yaw) cos(second.yaw)];
overlap = true;
for k = 1:size(axes,1)
    axisVector = axes(k,:);
    firstProjection = firstCorners*axisVector';
    secondProjection = secondCorners*axisVector';
    if max(firstProjection) < min(secondProjection) || ...
            max(secondProjection) < min(firstProjection)
        overlap = false;
        break;
    end
end
if overlap
    clearance = 0;
else
    clearance = polygonDistance(firstCorners, secondCorners);
end
end

function time = sweptCollisionTime(first, second, firstVelocity, ...
        secondVelocity, horizon, overlapsNow)
if overlapsNow
    time = 0;
    return;
end
relativePosition = [second.x-first.x second.y-first.y];
relativeVelocity = secondVelocity-firstVelocity;
axes = [cos(first.yaw) sin(first.yaw); ...
    -sin(first.yaw) cos(first.yaw); cos(second.yaw) sin(second.yaw); ...
    -sin(second.yaw) cos(second.yaw)];
entry = -inf;
exit = inf;
for k = 1:size(axes,1)
    axisVector = axes(k,:);
    center = dot(relativePosition, axisVector);
    speed = dot(relativeVelocity, axisVector);
    radius = projectionRadius(first, axisVector) + ...
        projectionRadius(second, axisVector);
    if abs(speed) <= eps
        if abs(center) > radius
            time = inf;
            return;
        end
    else
        firstTime = (-radius-center)/speed;
        secondTime = (radius-center)/speed;
        entry = max(entry, min(firstTime, secondTime));
        exit = min(exit, max(firstTime, secondTime));
        if entry > exit
            time = inf;
            return;
        end
    end
end
entry = max(entry, 0);
if exit >= entry && entry <= horizon
    time = entry;
else
    time = inf;
end
end

function radius = projectionRadius(rectangle, axisVector)
heading = [cos(rectangle.yaw) sin(rectangle.yaw)];
left = [-sin(rectangle.yaw) cos(rectangle.yaw)];
radius = rectangle.length/2*abs(dot(heading, axisVector)) + ...
    rectangle.width/2*abs(dot(left, axisVector));
end

function corners = rectangleCorners(rectangle)
localCorners = [rectangle.length/2 rectangle.width/2; ...
    -rectangle.length/2 rectangle.width/2; ...
    -rectangle.length/2 -rectangle.width/2; ...
    rectangle.length/2 -rectangle.width/2];
rotation = [cos(rectangle.yaw) -sin(rectangle.yaw); ...
    sin(rectangle.yaw) cos(rectangle.yaw)];
corners = localCorners*rotation' + [rectangle.x rectangle.y];
end

function distance = polygonDistance(first, second)
distance = inf;
for i = 1:size(first,1)
    firstNext = mod(i, size(first,1)) + 1;
    for j = 1:size(second,1)
        secondNext = mod(j, size(second,1)) + 1;
        if segmentsIntersect(first(i,:), first(firstNext,:), ...
                second(j,:), second(secondNext,:))
            distance = 0;
            return;
        end
        distance = min(distance, pointSegmentDistance( ...
            first(i,:), second(j,:), second(secondNext,:)));
        distance = min(distance, pointSegmentDistance( ...
            second(j,:), first(i,:), first(firstNext,:)));
    end
end
end

function distance = pointSegmentDistance(point, startPoint, endPoint)
segment = endPoint-startPoint;
denominator = dot(segment, segment);
if denominator <= eps
    distance = hypot(point(1)-startPoint(1), point(2)-startPoint(2));
else
    fraction = min(1, max(0, dot(point-startPoint, segment)/denominator));
    closest = startPoint + fraction*segment;
    distance = hypot(point(1)-closest(1), point(2)-closest(2));
end
end

function intersects = segmentsIntersect(a, b, c, d)
ab = b-a;
cd = d-c;
denominator = cross2(ab, cd);
if abs(denominator) <= eps
    intersects = false;
    return;
end
fractionA = cross2(c-a, cd)/denominator;
fractionB = cross2(c-a, ab)/denominator;
intersects = fractionA >= 0 && fractionA <= 1 && ...
    fractionB >= 0 && fractionB <= 1;
end

function value = cross2(first, second)
value = first(1)*second(2)-first(2)*second(1);
end
