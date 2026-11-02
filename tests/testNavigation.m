function tests = testNavigation()
%TESTNAVIGATION Unit tests for Stage 1 global navigation.
tests = functiontests(localfunctions);
end

function testAStarAndDijkstraAgree(testCase)
cfg = defaultConfig();
G = buildRoadGraph(cfg);
[astarNodes, astarCost, astarWaypoints, astarFound] = ...
    planGlobalRoute(G, 1, 25, "astar");
[dijkstraNodes, dijkstraCost, dijkstraWaypoints, dijkstraFound] = ...
    planGlobalRoute(G, 1, 25, "dijkstra");
verifyTrue(testCase, astarFound);
verifyTrue(testCase, dijkstraFound);
verifyEqual(testCase, astarCost, dijkstraCost, 'AbsTol', 1e-10);
verifyEqual(testCase, astarNodes(1), 1);
verifyEqual(testCase, astarNodes(end), 25);
verifyEqual(testCase, astarWaypoints, dijkstraWaypoints);
end

function testUnreachableGoalReturnsFlag(testCase)
cfg = defaultConfig();
G = buildRoadGraph(cfg);
G = addedge(G, 26, 27);
G.Nodes.X(26:27) = [100; 110];
G.Nodes.Y(26:27) = [100; 100];
G.Edges.Length(end) = 10;
G.Edges.SpeedLimit(end) = cfg.nav.localSpeedLimit;
G.Edges.Roughness(end) = cfg.nav.localRoughness;
G.Edges.RoadType(end) = "isolated";
[nodes, cost, waypoints, found] = planGlobalRoute(G, 1, 26, "astar");
verifyFalse(testCase, found);
verifyEmpty(testCase, nodes);
verifyTrue(testCase, isinf(cost));
verifyEmpty(testCase, waypoints);
end

function testDensifiedSpacing(testCase)
cfg = defaultConfig();
route = densifyRoute([0 0; 12 0; 20 8], cfg);
spacing = hypot(diff(route.x), diff(route.y));
verifyGreaterThan(testCase, numel(spacing), 2);
verifyLessThanOrEqual(testCase, max(spacing), ...
    cfg.nav.routeSpacing + cfg.nav.splineResolution);
verifyEqual(testCase, spacing(1:end-1), ...
    repmat(cfg.nav.routeSpacing, numel(spacing)-1, 1), ...
    'AbsTol', cfg.nav.splineResolution);
verifyEqual(testCase, numel(route.x), numel(route.curvature));
end

function testLocalReferenceProgressIsMonotonic(testCase)
cfg = defaultConfig();
route = densifyRoute([0 0; 60 0], cfg);
lastIndex = 1;
previousIndex = 0;
for x = [0 8 16 24 32 40 48]
    ego = types.egoState();
    ego.x = x;
    [nearestIndex, reference, crossTrackError] = ...
        getLocalReference(ego, route, lastIndex, cfg);
    verifyGreaterThanOrEqual(testCase, nearestIndex, previousIndex);
    verifyGreaterThan(testCase, numel(reference.x), 0);
    verifyEqual(testCase, crossTrackError, 0, 'AbsTol', 1e-10);
    previousIndex = nearestIndex;
    lastIndex = nearestIndex;
end
end
