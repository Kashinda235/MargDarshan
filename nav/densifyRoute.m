function route = densifyRoute(waypoints, cfg)
%DENSIFYROUTE Smooth and resample a waypoint polyline at fixed spacing.
% Inputs: waypoints, finite N-by-2 [x y] world coordinates (m); cfg,
% defaultConfig output (optional).
% Outputs: route with x/y (m), s (m), heading (rad), and curvature (1/m)
% columns, sampled at cfg.nav.routeSpacing except for a shorter final gap.
% Example: route = densifyRoute([0 0; 10 0; 15 5], defaultConfig());

if nargin < 2
    cfg = defaultConfig();
end
validateattributes(waypoints, {'numeric'}, ...
    {'2d','ncols',2,'nonempty','real','finite'}, mfilename, 'waypoints');
spacing = cfg.nav.routeSpacing;
resolution = cfg.nav.splineResolution;
validateattributes(spacing, {'numeric'}, {'scalar','positive','finite'});
validateattributes(resolution, {'numeric'}, {'scalar','positive','finite'});

segmentLength = hypot(diff(waypoints(:,1)), diff(waypoints(:,2)));
keep = [true; segmentLength > eps(max(abs(waypoints(:))))];
waypoints = waypoints(keep, :);
if size(waypoints, 1) == 1
    route = struct('x', waypoints(1,1), 'y', waypoints(1,2), ...
        's', 0, 'heading', 0, 'curvature', 0);
    return;
end

originalS = [0; cumsum(hypot(diff(waypoints(:,1)), diff(waypoints(:,2))))];
if numel(originalS) > 2
    fineS = unique([0:resolution:originalS(end), originalS(end)]);
    fineX = spline(originalS, waypoints(:,1), fineS);
    fineY = spline(originalS, waypoints(:,2), fineS);
else
    fineS = unique([0:resolution:originalS(end), originalS(end)]);
    fineX = interp1(originalS, waypoints(:,1), fineS);
    fineY = interp1(originalS, waypoints(:,2), fineS);
end
fineDistance = [0; cumsum(hypot(diff(fineX(:)), diff(fineY(:))))];
valid = [true; diff(fineDistance) > eps(max(fineDistance))];
fineDistance = fineDistance(valid);
fineX = fineX(valid);
fineY = fineY(valid);
sampleS = unique([0:spacing:fineDistance(end), fineDistance(end)])';
sampleX = interp1(fineDistance, fineX, sampleS, 'linear');
sampleY = interp1(fineDistance, fineY, sampleS, 'linear');

sampleCount = numel(sampleS);
heading = zeros(sampleCount, 1);
if sampleCount > 1
    for k = 1:sampleCount
        before = max(1, k - 1);
        after = min(sampleCount, k + 1);
        heading(k) = atan2(sampleY(after) - sampleY(before), ...
            sampleX(after) - sampleX(before));
    end
end
curvature = zeros(sampleCount, 1);
if sampleCount > 2
    headingChange = atan2(sin(diff(heading)), cos(diff(heading)));
    ds = diff(sampleS);
    curvature(2:end-1) = ...
        (headingChange(1:end-1) + headingChange(2:end)) ./ ...
        (ds(1:end-1) + ds(2:end));
    curvature(1) = headingChange(1) / ds(1);
    curvature(end) = headingChange(end) / ds(end);
end
route = struct('x', sampleX(:), 'y', sampleY(:), 's', sampleS(:), ...
    'heading', heading, 'curvature', curvature);
end
