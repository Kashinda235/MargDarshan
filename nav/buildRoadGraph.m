function G = buildRoadGraph(cfg)
%BUILDROADGRAPH Build the built-in demo road network as an undirected graph.
% Inputs: cfg, defaultConfig output (optional).
% Outputs: G, a graph with node X/Y coordinates and edge Length (m),
% SpeedLimit (m/s), RoadType, and Roughness attributes.
% Example: G = buildRoadGraph(defaultConfig());

if nargin < 1
    cfg = defaultConfig();
end

x = [0; 10; 20; 30; 40; 50; ...
     0; 10; 20; 30; 40; ...
     0; 10; 20; 30; 40; 50; 60; ...
     30; 30; ...
     30; 35; 40; 45; 50];
y = [0; 0; 0; 0; 0; 0; ...
    -10; -10; -10; -10; -10; ...
     30; 30; 30; 30; 30; 30; 30; ...
     12; 22; ...
    -10; -15; -15; -10; -10];
feature = repmat("road", numel(x), 1);
feature(4) = "unsignalized_intersection";
feature(15) = "highway_merge";
feature(23) = "market";
nodeTable = table(x, y, feature, ...
    'VariableNames', {'X', 'Y', 'Feature'});

% Main village road, parallel village road, highway, merge ramp, and market.
source = [1 2 3 4 5, 1 7 8 9 10 11, ...
          12 13 14 15 16 17, 4 19 20, ...
          4 21 22 23 24 25];
target = [2 3 4 5 6, 7 8 9 10 11 6, ...
          13 14 15 16 17 18, 19 20 15, ...
          21 22 23 24 25 6];
roadType = [repmat("village_road", 1, 2), ...
            repmat("unsignalized_intersection", 1, 2), ...
            repmat("village_road", 1, 1), ...
            repmat("village_road", 1, 6), ...
            repmat("highway", 1, 6), ...
            repmat("highway_merge", 1, 3), ...
            repmat("market_street", 1, 6)]';

speedLimit = zeros(numel(source), 1);
roughness = zeros(numel(source), 1);
for k = 1:numel(source)
    switch roadType(k)
        case "village_road"
            speedLimit(k) = cfg.nav.villageSpeedLimit;
            roughness(k) = cfg.nav.villageRoughness;
        case "unsignalized_intersection"
            speedLimit(k) = cfg.nav.localSpeedLimit;
            roughness(k) = cfg.nav.localRoughness;
        case "highway"
            speedLimit(k) = cfg.nav.highwaySpeedLimit;
            roughness(k) = cfg.nav.highwayRoughness;
        case "highway_merge"
            speedLimit(k) = cfg.nav.localSpeedLimit;
            roughness(k) = cfg.nav.mergeRoughness;
        case "market_street"
            speedLimit(k) = cfg.nav.marketSpeedLimit;
            roughness(k) = cfg.nav.marketRoughness;
    end
end
lengthM = hypot(x(source) - x(target), y(source) - y(target));
endNodes = [source(:), target(:)];
edgeTable = table(endNodes, lengthM, speedLimit, roadType, roughness, ...
    'VariableNames', {'EndNodes', 'Length', 'SpeedLimit', ...
    'RoadType', 'Roughness'});
G = graph(edgeTable, nodeTable);
end
