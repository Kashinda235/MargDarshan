function corridor = pocketCorridor(pocketState, riskField, cfg)
%POCKETCORRIDOR Extract a fixed-capacity forward corridor from a pocket.
% Inputs: pocketState, connected pocket contract; riskField, axes from
% computeRiskField; cfg, defaults. Coordinates and widths are in m.
% Outputs: corridor with fixed arrays for centreline [x y], x stations,
% left/right widths, and populated sample count.
% Example: corridor = pocketCorridor(pocket, field, defaultConfig());

if nargin < 3 || isempty(cfg)
    cfg = defaultConfig();
end
if ~isstruct(pocketState) || ~isfield(pocketState, 'mask') || ...
        ~isequal(size(pocketState.mask), [cfg.grid.ny cfg.grid.nx]) || ...
        ~isstruct(riskField) || ~isfield(riskField, 'localX') || ...
        ~isfield(riskField, 'localY') || ...
        numel(riskField.localX) ~= cfg.grid.nx || ...
        numel(riskField.localY) ~= cfg.grid.ny
    error('pocketCorridor:InvalidInput', ...
        'pocketState and riskField must match the configured grid.');
end
capacity = cfg.grid.nx;
corridor = struct( ...
    'count', 0, ...
    'centerline', zeros(capacity, 2), ...
    'x', zeros(capacity, 1), ...
    'centerY', zeros(capacity, 1), ...
    'leftWidth', zeros(capacity, 1), ...
    'rightWidth', zeros(capacity, 1));
if pocketState.isEmpty
    return;
end

[~, egoColumn] = min(abs(riskField.localX));
[~, previousRow] = min(abs(riskField.localY));
for column = egoColumn:cfg.grid.nx
    columnMask = pocketState.mask(:, column);
    transitions = diff([false; columnMask; false]);
    starts = find(transitions == 1);
    ends = find(transitions == -1)-1;
    if isempty(starts)
        break;
    end
    intervalCenters = (starts+ends)/2;
    [~, selected] = min(abs(riskField.localY(round(intervalCenters)) ...
        - riskField.localY(previousRow)));
    selectedRows = starts(selected):ends(selected);
    firstRow = selectedRows(1);
    lastRow = selectedRows(end);
    centerRow = (firstRow+lastRow)/2;
    centerY = (riskField.localY(firstRow) ...
        + riskField.localY(lastRow))/2;
    index = corridor.count+1;
    corridor.count = index;
    corridor.x(index) = riskField.localX(column);
    corridor.centerY(index) = centerY;
    corridor.centerline(index,:) = [corridor.x(index) centerY];
    corridor.leftWidth(index) = max(0, ...
        riskField.localY(lastRow)+cfg.grid.resolution/2-centerY);
    corridor.rightWidth(index) = max(0, ...
        centerY-riskField.localY(firstRow)+cfg.grid.resolution/2);
    previousRow = round(centerRow);
end
end
