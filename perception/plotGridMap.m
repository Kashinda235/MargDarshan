function ax = plotGridMap(map, layer, ax)
%PLOTGRIDMAP Display a selected ego-centric grid-map layer.
% Inputs: map, types.gridMap with [ny x nx] layers; layer, field name
% (default 'traversability'); ax, optional axes handle.
% Outputs: ax, populated axes; spatial coordinates are ego-frame m.
% Example: plotGridMap(map, 'occupancy');

if nargin < 2 || isempty(layer)
    layer = 'traversability';
end
if nargin < 3 || isempty(ax)
    ax = gca;
end
if isstring(layer) && isscalar(layer)
    layer = char(layer);
end
validLayers = {'occupancy','height','obstacleProb', ...
    'traversability','surfaceCost'};
if ~ischar(layer) || ~any(strcmp(layer, validLayers))
    error('plotGridMap:InvalidLayer', ...
        'layer must be one of: %s.', strjoin(validLayers, ', '));
end
switch layer
    case 'occupancy'
        values = map.occupancy;
    case 'height'
        values = map.height;
    case 'obstacleProb'
        values = map.obstacleProb;
    case 'traversability'
        values = map.traversability;
    otherwise
        values = map.surfaceCost;
end
if ~isequal(size(values), [map.ny map.nx])
    error('plotGridMap:InvalidMapSize', ...
        'Selected layer must have size [map.ny x map.nx].');
end
localX = map.originX+(0:map.nx-1)*map.resolution;
localY = map.originY+(0:map.ny-1)*map.resolution;
imagesc(ax, localX, localY, values);
set(ax, 'YDir', 'normal');
axis(ax, 'equal');
axis(ax, 'tight');
grid(ax, 'on');
xlabel(ax, 'Forward x (m)');
ylabel(ax, 'Left y (m)');
title(ax, sprintf('%s | t = %.2f s', layer, map.timestamp), ...
    'Interpreter', 'none');
colorbar(ax);
if ~strcmp(layer, 'height')
    caxis(ax, [0 1]);
end
end
