function forces = boidForces(agent, neighbours, ego, roadBoundaries, cfg)
%BOIDFORCES Compute configured neighbour, road, and flow accelerations.
% Inputs: agent and neighbours, world-frame agent records; ego, world-frame
% ego pose/speed; roadBoundaries, polygon vertices in m or []; cfg, defaults.
% Outputs: struct of weighted acceleration terms in world-frame m/s^2.
% Example: f = boidForces(a, agents, ego, polygon, defaultConfig());

if nargin < 5 || isempty(cfg)
    cfg = defaultConfig();
end
if nargin < 4
    roadBoundaries = [];
end
model = agentTypeModel(cfg, agent.type);
position = [agent.x agent.y];
velocity = [agent.vx agent.vy];
separation = [0 0];
alignmentSum = [0 0];
alignmentWeightSum = 0;
cohesionSum = [0 0];
cohesionCount = 0;
for k = 1:neighbours.count
    neighbour = neighbours.agents(k);
    if ~neighbour.valid || neighbour.id == agent.id
        continue;
    end
    [separation, alignmentSum, alignmentWeightSum, ...
        cohesionSum, cohesionCount] = accumulateNeighbour( ...
        agent, neighbour, position, model, cfg, ...
        separation, alignmentSum, alignmentWeightSum, ...
        cohesionSum, cohesionCount);
end
egoNeighbour = struct( ...
    'id', -1, 'type', cfg.agentTypes.car, ...
    'x', ego.x, 'y', ego.y, ...
    'vx', ego.v*cos(ego.yaw), 'vy', ego.v*sin(ego.yaw), ...
    'valid', true);
[separation, alignmentSum, alignmentWeightSum, ...
    cohesionSum, cohesionCount] = accumulateNeighbour( ...
    agent, egoNeighbour, position, model, cfg, ...
    separation, alignmentSum, alignmentWeightSum, ...
    cohesionSum, cohesionCount);

alignment = [0 0];
cohesion = [0 0];
if alignmentWeightSum > 0
    alignment = alignmentSum/alignmentWeightSum - velocity;
end
if cohesionCount > 0
    centroidDelta = cohesionSum/cohesionCount - position;
    centroidDistance = hypot(centroidDelta(1), centroidDelta(2));
    if centroidDistance > cfg.prediction.forceEpsilonM
        cohesion = centroidDelta * min( ...
            centroidDistance/cfg.prediction.cohesionRadiusM, 1) ...
            / centroidDistance;
    end
end
boundary = boundaryRepulsion(position, roadBoundaries, cfg);
speed = hypot(velocity(1), velocity(2));
heading = [cos(agent.yaw) sin(agent.yaw)];
goalAttraction = speed*heading - velocity;
weights = model.boidWeights;
forces = struct( ...
    'separation', weights(1)*separation, ...
    'alignment', weights(2)*alignment, ...
    'cohesion', weights(3)*cohesion, ...
    'roadBoundary', weights(4)*boundary, ...
    'goalAttraction', weights(5)*goalAttraction);
end

function [separation, alignmentSum, alignmentWeightSum, ...
        cohesionSum, cohesionCount] = accumulateNeighbour( ...
        agent, neighbour, position, model, cfg, ...
        separation, alignmentSum, alignmentWeightSum, ...
        cohesionSum, cohesionCount)
delta = position - [neighbour.x neighbour.y];
distance = hypot(delta(1), delta(2));
neighbourModel = agentTypeModel(cfg, neighbour.type);
radius = max(model.separationRadiusM, neighbourModel.separationRadiusM);
if distance < radius
    if distance > cfg.prediction.forceEpsilonM
        direction = delta/distance;
    else
        direction = -[cos(agent.yaw) sin(agent.yaw)];
    end
    separation = separation + direction*((radius-distance)/radius);
end
if distance <= cfg.prediction.cohesionRadiusM
    weight = 1/max(distance, cfg.prediction.alignmentDistanceFloorM);
    alignmentSum = alignmentSum + ...
        weight*[neighbour.vx neighbour.vy];
    alignmentWeightSum = alignmentWeightSum + weight;
    cohesionSum = cohesionSum + [neighbour.x neighbour.y];
    cohesionCount = cohesionCount + 1;
end
end

function force = boundaryRepulsion(position, polygon, cfg)
force = [0 0];
if isempty(polygon) || size(polygon, 1) < 3
    return;
end
inside = inpolygon(position(1), position(2), polygon(:,1), polygon(:,2));
bestDistance = inf;
nearestPoint = [0 0];
nearestEdge = [0 0];
for k = 1:size(polygon, 1)
    next = mod(k, size(polygon, 1)) + 1;
    startPoint = polygon(k, :);
    edge = polygon(next, :) - startPoint;
    denominator = dot(edge, edge);
    if denominator <= cfg.prediction.forceEpsilonM^2
        continue;
    end
    fraction = min(max(dot(position-startPoint, edge)/denominator, 0), 1);
    point = startPoint + fraction*edge;
    distance = hypot(position(1)-point(1), position(2)-point(2));
    if distance < bestDistance
        bestDistance = distance;
        nearestPoint = point;
        nearestEdge = edge;
    end
end
if bestDistance >= cfg.prediction.boundaryInfluenceM
    return;
end
if bestDistance > cfg.prediction.forceEpsilonM
    if inside
        signedArea = sum(polygon(:,1).*polygon([2:end 1],2) - ...
            polygon([2:end 1],1).*polygon(:,2));
        orientation = sign(signedArea);
        force = orientation*[-nearestEdge(2) nearestEdge(1)] ...
            /hypot(nearestEdge(1), nearestEdge(2));
    else
        delta = nearestPoint - position;
        force = delta/bestDistance;
    end
else
    signedArea = sum(polygon(:,1).*polygon([2:end 1],2) - ...
        polygon([2:end 1],1).*polygon(:,2));
    orientation = sign(signedArea);
    force = orientation*[-nearestEdge(2) nearestEdge(1)] ...
        /max(hypot(nearestEdge(1), nearestEdge(2)), ...
             cfg.prediction.forceEpsilonM);
end
force = force*((cfg.prediction.boundaryInfluenceM-bestDistance) ...
    /cfg.prediction.boundaryInfluenceM);
end
