function [nearestIndex, reference, crossTrackError] = ...
    getLocalReference(egoState, denseRoute, lastIndex, cfg)
%GETLOCALREFERENCE Find a forward-only local route reference in ego frame.
% Inputs: egoState with x/y (m) and yaw (rad); denseRoute from
% densifyRoute; lastIndex, prior route index (optional); cfg,
% defaultConfig output (optional).
% Outputs: nearestIndex, monotonic nearest dense-route index; reference,
% struct of next x/y (m), relative heading (rad), and curvature (1/m) in
% ego frame; crossTrackError, signed lateral offset (m), positive left.
% Example: [idx,ref,cte] = getLocalReference(types.egoState(),route,1);

if nargin < 3 || isempty(lastIndex)
    lastIndex = 1;
end
if nargin < 4
    cfg = defaultConfig();
end
validateattributes(egoState.x, {'numeric'}, {'scalar','real','finite'});
validateattributes(egoState.y, {'numeric'}, {'scalar','real','finite'});
validateattributes(egoState.yaw, {'numeric'}, {'scalar','real','finite'});
routeCount = numel(denseRoute.x);
if routeCount == 0 || any([numel(denseRoute.y), numel(denseRoute.heading), ...
        numel(denseRoute.curvature)] ~= routeCount)
    error('getLocalReference:InvalidRoute', ...
        'denseRoute must contain equally sized, nonempty x, y, heading, and curvature arrays.');
end
validateattributes(lastIndex, {'numeric'}, ...
    {'scalar','integer','>=',1,'<=',routeCount});

searchEnd = min(routeCount, lastIndex + cfg.nav.searchWindow);
searchIndices = lastIndex:searchEnd;
dx = denseRoute.x(searchIndices) - egoState.x;
dy = denseRoute.y(searchIndices) - egoState.y;
[~, localIndex] = min(dx.^2 + dy.^2);
nearestIndex = searchIndices(localIndex);

routeHeading = denseRoute.heading(nearestIndex);
offsetX = egoState.x - denseRoute.x(nearestIndex);
offsetY = egoState.y - denseRoute.y(nearestIndex);
crossTrackError = -sin(routeHeading) .* offsetX + cos(routeHeading) .* offsetY;

lastReference = min(routeCount, ...
    nearestIndex + cfg.nav.referencePointCount - 1);
referenceIndices = nearestIndex:lastReference;
relativeX = denseRoute.x(referenceIndices) - egoState.x;
relativeY = denseRoute.y(referenceIndices) - egoState.y;
cosYaw = cos(egoState.yaw);
sinYaw = sin(egoState.yaw);
reference = struct( ...
    'x', cosYaw .* relativeX + sinYaw .* relativeY, ...
    'y', -sinYaw .* relativeX + cosYaw .* relativeY, ...
    'heading', atan2(sin(denseRoute.heading(referenceIndices) - egoState.yaw), ...
        cos(denseRoute.heading(referenceIndices) - egoState.yaw)), ...
    'curvature', denseRoute.curvature(referenceIndices));
end
