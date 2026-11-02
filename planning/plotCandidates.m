function ax = plotCandidates(candidates, candidateCount, selectedIndex, ax)
%PLOTCANDIDATES Plot world-frame paths by cost, grey rejected, bold selected.
% Inputs: fixed candidate array; count; selectedIndex (0 if fallback);
% ax, optional axes handle. Coordinates are ENU m.
% Outputs: ax, axes handle.
% Example: plotCandidates(debug.candidates,n,d.selectedIndex,gca);

if nargin < 4 || isempty(ax)
    ax = gca;
end
if candidateCount < 0 || candidateCount > numel(candidates) || ...
        candidateCount ~= floor(candidateCount)
    error('plotCandidates:InvalidCandidateCount', ...
        'candidateCount must be within the fixed candidate array.');
end
if selectedIndex < 0 || selectedIndex > candidateCount || ...
        selectedIndex ~= floor(selectedIndex)
    error('plotCandidates:InvalidSelectedIndex', ...
        'selectedIndex must be zero or an active candidate index.');
end
hold(ax, 'on');
costs = [candidates(1:candidateCount).cost];
finiteCosts = costs(isfinite(costs));
if isempty(finiteCosts)
    minimumCost = 0;
    maximumCost = 1;
else
    minimumCost = min(finiteCosts);
    maximumCost = max(finiteCosts);
    if maximumCost <= minimumCost
        maximumCost = minimumCost+1;
    end
end
colorMap = parula(64);
for k = 1:candidateCount
    path = candidates(k);
    if ~path.feasible
        plot(ax, path.x, path.y, '-', 'Color', [0.65 0.65 0.65], ...
            'LineWidth', 0.7);
    else
        colorIndex = 1+round((size(colorMap,1)-1)* ...
            (path.cost-minimumCost)/(maximumCost-minimumCost));
        colorIndex = min(size(colorMap,1), max(1, colorIndex));
        lineWidth = 0.8;
        if k == selectedIndex
            lineWidth = 2.8;
        end
        plot(ax, path.x, path.y, '-', 'Color', colorMap(colorIndex,:), ...
            'LineWidth', lineWidth);
    end
end
if selectedIndex > 0
    selected = candidates(selectedIndex);
    plot(ax, selected.x, selected.y, 'k-', 'LineWidth', 2.8);
end
axis(ax, 'equal');
grid(ax, 'on');
xlabel(ax, 'East x (m)');
ylabel(ax, 'North y (m)');
hold(ax, 'off');
end
