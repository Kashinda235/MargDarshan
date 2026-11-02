%DEMONAV Plot the demo road graph and its computed global route.
% Example: run('nav/demoNav.m')

cfg = defaultConfig();
G = buildRoadGraph(cfg);
[nodeSequence, totalCost, waypoints, routeFound] = ...
    planGlobalRoute(G, 1, 23, "astar");
if ~routeFound
    error('demoNav:RouteNotFound', 'No route exists between the demo nodes.');
end
figure('Name', 'MargDarshan global navigation demo');
plotRoute(G, waypoints);
fprintf('Route nodes: %s\n', mat2str(nodeSequence'));
fprintf('Weighted travel cost: %.2f s\n', totalCost);
