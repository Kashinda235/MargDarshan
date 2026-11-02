function [nodeSequence, totalCost, waypoints, routeFound] = ...
    planGlobalRoute(G, startNode, goalNode, method)
%PLANGLOBALROUTE Find a minimum travel-cost route using A* or Dijkstra.
% Inputs: G, a graph/digraph from buildRoadGraph; startNode and goalNode,
% integer node indices; method, "astar" (default) or "dijkstra".
% Outputs: nodeSequence, ordered node indices; totalCost, estimated travel
% time weighted by roughness (s); waypoints, [x y] coordinates (m);
% routeFound, false with empty outputs when no path exists.
% Example: [nodes,cost,xy,found] = planGlobalRoute(G,1,25,"astar");

if nargin < 4 || isempty(method)
    method = "astar";
end
method = lower(string(method));
if ~isscalar(method) || ~any(method == ["astar", "a*", "dijkstra"])
    error('planGlobalRoute:InvalidMethod', ...
        'Method must be "astar" or "dijkstra".');
end
nodeCount = numnodes(G);
validateattributes(startNode, {'numeric'}, ...
    {'scalar','integer','>=',1,'<=',nodeCount}, mfilename, 'startNode');
validateattributes(goalNode, {'numeric'}, ...
    {'scalar','integer','>=',1,'<=',nodeCount}, mfilename, 'goalNode');
requiredEdgeFields = {'Length', 'SpeedLimit', 'Roughness'};
if ~all(ismember(requiredEdgeFields, G.Edges.Properties.VariableNames)) || ...
        ~all(ismember({'X','Y'}, G.Nodes.Properties.VariableNames))
    error('planGlobalRoute:InvalidGraph', ...
        'G must contain edge Length, SpeedLimit, Roughness and node X, Y attributes.');
end

if startNode == goalNode
    nodeSequence = startNode;
    totalCost = 0;
    waypoints = [G.Nodes.X(startNode), G.Nodes.Y(startNode)];
    routeFound = true;
    return;
end
if numedges(G) == 0
    nodeSequence = zeros(0, 1);
    totalCost = inf;
    waypoints = zeros(0, 2);
    routeFound = false;
    return;
end
edgeCost = G.Edges.Length ./ G.Edges.SpeedLimit .* G.Edges.Roughness;
if any(~isfinite(edgeCost) | edgeCost <= 0)
    error('planGlobalRoute:InvalidGraph', ...
        'Edge lengths, speed limits, and roughness must produce positive finite costs.');
end
endNodes = G.Edges.EndNodes;
edgeDistance = hypot( ...
    G.Nodes.X(endNodes(:,1)) - G.Nodes.X(endNodes(:,2)), ...
    G.Nodes.Y(endNodes(:,1)) - G.Nodes.Y(endNodes(:,2)));
if any(edgeDistance == 0)
    heuristicScale = 0;
else
    heuristicScale = min(edgeCost ./ edgeDistance);
end
costToNode = inf(nodeCount, 1);
costToNode(startNode) = 0;
parent = zeros(nodeCount, 1);
open = false(nodeCount, 1);
closed = false(nodeCount, 1);
open(startNode) = true;
routeFound = false;

while any(open)
    candidates = find(open);
    if method == "dijkstra"
        priority = costToNode(candidates);
    else
        dx = G.Nodes.X(candidates) - G.Nodes.X(goalNode);
        dy = G.Nodes.Y(candidates) - G.Nodes.Y(goalNode);
        priority = costToNode(candidates) + heuristicScale .* hypot(dx, dy);
    end
    [~, best] = min(priority);
    current = candidates(best);
    open(current) = false;
    if current == goalNode
        routeFound = true;
        break;
    end
    closed(current) = true;

    adjacent = neighbors(G, current);
    for k = 1:numel(adjacent)
        next = adjacent(k);
        if closed(next)
            continue;
        end
        edgeIndex = findedge(G, current, next);
        nextCost = costToNode(current) + min(edgeCost(edgeIndex));
        if nextCost < costToNode(next)
            costToNode(next) = nextCost;
            parent(next) = current;
            open(next) = true;
        end
    end
end

if ~routeFound
    nodeSequence = zeros(0, 1);
    totalCost = inf;
    waypoints = zeros(0, 2);
    return;
end

nodeSequence = goalNode;
while nodeSequence(1) ~= startNode
    predecessor = parent(nodeSequence(1));
    if predecessor == 0
        error('planGlobalRoute:PathReconstructionFailed', ...
            'A route was found but its predecessor chain is incomplete.');
    end
    nodeSequence = [predecessor; nodeSequence]; %#ok<AGROW>
end
totalCost = costToNode(goalNode);
waypoints = [G.Nodes.X(nodeSequence), G.Nodes.Y(nodeSequence)];
end
