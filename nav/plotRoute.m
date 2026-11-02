
function ax = plotRoute(G, waypoints)
%PLOTROUTE Plot a road graph and optionally overlay a route polyline.
% Inputs: G, graph/digraph with node X/Y attributes; waypoints, optional
% N-by-2 [x y] world coordinates (m).
% Outputs: ax, axes handle for the plot.
% Example: plotRoute(G, routeWaypoints);

if nargin < 2
    waypoints = zeros(0, 2);
end
if ~all(ismember({'X','Y'}, G.Nodes.Properties.VariableNames))
    error('plotRoute:InvalidGraph', ...
        'G must contain node X and Y coordinate attributes.');
end
ax = gca;
plot(ax, G, 'XData', G.Nodes.X, 'YData', G.Nodes.Y, ...
    'NodeLabel', string(1:numnodes(G)), 'LineWidth', 1.2);
hold(ax, 'on');
if ~isempty(waypoints)
    validateattributes(waypoints, {'numeric'}, ...
        {'2d','ncols',2,'real','finite'}, mfilename, 'waypoints');
    plot(ax, waypoints(:,1), waypoints(:,2), 'r-', 'LineWidth', 2.5);
    plot(ax, waypoints(1,1), waypoints(1,2), 'go', ...
        'MarkerFaceColor', 'g', 'MarkerSize', 7);
    plot(ax, waypoints(end,1), waypoints(end,2), 'mo', ...
        'MarkerFaceColor', 'm', 'MarkerSize', 7);
end
axis(ax, 'equal');
grid(ax, 'on');
xlabel(ax, 'East x (m)');
ylabel(ax, 'North y (m)');
title(ax, 'MargDarshan demo road network');
hold(ax, 'off');
end
