function pocketState = generateSafePocket(riskField, cfg)
%GENERATESAFEPOCKET Flood-fill the safe connected region around ego.
% Inputs: riskField from computeRiskField; cfg, defaults including the risk
% threshold and maximum boundary size. Grid coordinates are ego-frame m.
% Outputs: fixed-size pocket contract with mask, simplified polygon (m),
% area (m^2), forward depth/lateral clearances (m), quality [0,1], and
% empty/small flags. Forward is local +x; left is local +y.
% Example: pocketState = generateSafePocket(field, defaultConfig());

if nargin < 2 || isempty(cfg)
    cfg = defaultConfig();
end
expectedGridSize = [cfg.grid.ny cfg.grid.nx];
if ~isstruct(riskField) || ~isfield(riskField, 'maxRisk') || ...
        ~isequal(size(riskField.maxRisk), expectedGridSize) || ...
        ~isfield(riskField, 'localX') || ...
        numel(riskField.localX) ~= cfg.grid.nx || ...
        ~isfield(riskField, 'localY') || ...
        numel(riskField.localY) ~= cfg.grid.ny
    error('generateSafePocket:InvalidRiskField', ...
        'riskField must have grid-sized maxRisk and local coordinate axes.');
end

pocketState = types.pocket(cfg);
safe = riskField.maxRisk < cfg.riskThreshold;
[~, egoColumn] = min(abs(riskField.localX));
[~, egoRow] = min(abs(riskField.localY));
if ~safe(egoRow, egoColumn)
    return;
end

startIndex = sub2ind(size(safe), egoRow, egoColumn);
pocketState.mask(startIndex) = true;
while true
    forwardNeighbours = false(size(safe));
    forwardNeighbours(:, 2:end) = pocketState.mask(:, 1:end-1);
    forwardNeighbours(2:end, 2:end) = ...
        forwardNeighbours(2:end, 2:end) ...
        | pocketState.mask(1:end-1, 1:end-1);
    forwardNeighbours(1:end-1, 2:end) = ...
        forwardNeighbours(1:end-1, 2:end) ...
        | pocketState.mask(2:end, 1:end-1);
    expanded = pocketState.mask ...
        | (safe & forwardNeighbours);
    expanded = expanded | (safe & ...
        (conv2(double(pocketState.mask), ones(3), 'same') > 0));
    if ~any(expanded(:) ~= pocketState.mask(:))
        break;
    end
    pocketState.mask = expanded;
end

pocketState.isEmpty = false;
pocketState.area = nnz(pocketState.mask)*cfg.grid.resolution^2;
pocketState.riskThreshold = cfg.riskThreshold;
[pocketState.boundary, pocketState.boundaryCount] = boundaryPolygon( ...
    pocketState.mask, riskField.localX, riskField.localY, cfg);

egoColumnMask = pocketState.mask(:, egoColumn);
egoRowRun = contiguousRun(egoColumnMask, egoRow);
if ~isempty(egoRowRun)
    pocketState.lateralLeft = max(0, ...
        riskField.localY(egoRowRun(end)) ...
        + cfg.grid.resolution/2-riskField.localY(egoRow));
    pocketState.lateralRight = max(0, ...
        riskField.localY(egoRow) ...
        - riskField.localY(egoRowRun(1))+cfg.grid.resolution/2);
end
forwardRows = pocketState.mask(egoRow, egoColumn:end);
firstBlocked = find(~forwardRows, 1);
if isempty(firstBlocked)
    lastForwardColumn = cfg.grid.nx;
else
    lastForwardColumn = egoColumn+firstBlocked-2;
end
if lastForwardColumn >= egoColumn
    pocketState.forwardDepth = max(0, ...
        riskField.localX(lastForwardColumn) ...
        - riskField.localX(egoColumn)+cfg.grid.resolution/2);
end
areaFraction = pocketState.area ...
    /(cfg.grid.ny*cfg.grid.nx*cfg.grid.resolution^2);
maximumForwardDepth = max(cfg.grid.resolution/2, ...
    riskField.localX(end)-riskField.localX(egoColumn) ...
    + cfg.grid.resolution/2);
depthFraction = min(1.0, ...
    pocketState.forwardDepth/maximumForwardDepth);
meanRisk = mean(riskField.maxRisk(pocketState.mask));
riskQuality = max(0.0, 1.0-meanRisk/cfg.riskThreshold);
pocketState.quality = min(1.0, max(0.0, ...
    areaFraction*depthFraction*riskQuality));
pocketState.isSmall = pocketState.quality ...
    < cfg.pocket.smallQualityThreshold;
end

function run = contiguousRun(values, startIndex)
runStart = startIndex;
runEnd = startIndex;
while runStart > 1 && values(runStart-1)
    runStart = runStart-1;
end
while runEnd < numel(values) && values(runEnd+1)
    runEnd = runEnd+1;
end
run = runStart:runEnd;
end

function [boundary, boundaryCount] = boundaryPolygon( ...
        mask, localX, localY, cfg)
maximumVertices = cfg.planner.maxBoundaryVertices;
boundary = zeros(maximumVertices, 2);
boundaryCount = 0;
if all(mask(:))
    vertices = [localX(1)-cfg.grid.resolution/2 ...
        localY(1)-cfg.grid.resolution/2; ...
        localX(end)+cfg.grid.resolution/2 ...
        localY(1)-cfg.grid.resolution/2; ...
        localX(end)+cfg.grid.resolution/2 ...
        localY(end)+cfg.grid.resolution/2; ...
        localX(1)-cfg.grid.resolution/2 ...
        localY(end)+cfg.grid.resolution/2];
else
    padded = false(size(mask)+2);
    padded(2:end-1, 2:end-1) = mask;
    contours = contourc(double(padded), [0.5 0.5]);
    largestArea = 0.0;
    vertices = zeros(0, 2);
    cursor = 1;
    while cursor < size(contours, 2)
        count = contours(2, cursor);
        contour = contours(:, cursor+1:cursor+count)';
        cursor = cursor+count+1;
        if count < 3
            continue;
        end
        candidate = [localX(1)+(contour(:,1)-2)*cfg.grid.resolution ...
            localY(1)+(contour(:,2)-2)*cfg.grid.resolution];
        candidateArea = polyarea(candidate(:,1), candidate(:,2));
        if candidateArea > largestArea
            largestArea = candidateArea;
            vertices = candidate;
        end
    end
end
if isempty(vertices)
    return;
end
vertices = simplifyPolygon(vertices, cfg.pocket.boundaryToleranceM);
if size(vertices, 1) > maximumVertices
    sampleIndices = round(linspace(1, size(vertices, 1), maximumVertices));
    vertices = vertices(sampleIndices, :);
end
boundary(1:size(vertices, 1), :) = vertices;
boundaryCount = size(vertices, 1);
end

function simplified = simplifyPolygon(vertices, tolerance)
if size(vertices, 1) < 4
    simplified = vertices;
    return;
end
if any(vertices(1,:) ~= vertices(end,:))
    vertices(end+1,:) = vertices(1,:);
end
keep = false(size(vertices, 1), 1);
keep([1 end]) = true;
stack = [1 size(vertices, 1)];
while ~isempty(stack)
    first = stack(end-1);
    last = stack(end);
    stack(end-1:end) = [];
    if last-first < 2
        continue;
    end
    segment = vertices(last,:)-vertices(first,:);
    segmentLengthSquared = sum(segment.^2);
    points = vertices(first+1:last-1,:);
    if segmentLengthSquared == 0
        distances = hypot(points(:,1)-vertices(first,1), ...
            points(:,2)-vertices(first,2));
    else
        fraction = ((points-vertices(first,:))*segment') ...
            / segmentLengthSquared;
        fraction = min(1.0, max(0.0, fraction));
        projection = vertices(first,:)+fraction.*segment;
        distances = hypot(points(:,1)-projection(:,1), ...
            points(:,2)-projection(:,2));
    end
    [maximumDistance, relativeIndex] = max(distances);
    if maximumDistance > tolerance
        split = first+relativeIndex;
        keep(split) = true;
        stack(end+1:end+2) = [first split];
        stack(end+1:end+2) = [split last];
    end
end
simplified = vertices(keep,:);
if size(simplified, 1) > 1 && ...
        all(simplified(1,:) == simplified(end,:))
    simplified(end,:) = [];
end
end
