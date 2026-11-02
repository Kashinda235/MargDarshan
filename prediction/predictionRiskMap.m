function [riskMap, localX, localY] = predictionRiskMap( ...
        predictions, ego, cfg)
%PREDICTIONRISKMAP Rasterise hypotheses into an ego-frame collision map.
% Inputs: predictions, fixed-capacity prediction list; ego, world-frame ego
% pose; cfg, defaults. Coordinates, agent dimensions, and sigma are in m.
% Outputs: riskMap [cfg.grid.nx x cfg.grid.ny x cfg.prediction.steps] with
% probabilities in [0,1], plus localX/localY [nx x ny] in ego-frame m.
% Example: [risk,x,y] = predictionRiskMap(p, ego, defaultConfig());

if nargin < 3 || isempty(cfg)
    cfg = defaultConfig();
end
if ~isstruct(predictions) || ~isfield(predictions, 'count') || ...
        ~isfield(predictions, 'agents') || predictions.count < 0 || ...
        predictions.count > cfg.maxAgents || ...
        numel(predictions.agents) ~= cfg.maxAgents
    error('predictionRiskMap:InvalidPredictions', ...
        'predictions must contain a count and cfg.maxAgents records.');
end
localAxisX = -((cfg.grid.nx-1)/2)*(cfg.grid.resolution) + ...
    (0:cfg.grid.nx-1)*cfg.grid.resolution;
localAxisY = -((cfg.grid.ny-1)/2)*(cfg.grid.resolution) + ...
    (0:cfg.grid.ny-1)*cfg.grid.resolution;
[localX, localY] = ndgrid(localAxisX, localAxisY);
riskMap = zeros(cfg.grid.nx, cfg.grid.ny, cfg.prediction.steps);
cosYaw = cos(ego.yaw);
sinYaw = sin(ego.yaw);
for k = 1:predictions.count
    prediction = predictions.agents(k);
    for step = 1:cfg.prediction.steps
        for mode = 1:prediction.hypothesisCount
            modeProbability = prediction.modeProbability(mode) * ...
                prediction.confidence;
            if modeProbability <= 0
                continue;
            end
            dxWorld = prediction.hypothesisX(step, mode)-ego.x;
            dyWorld = prediction.hypothesisY(step, mode)-ego.y;
            centerX = cosYaw*dxWorld + sinYaw*dyWorld;
            centerY = -sinYaw*dxWorld + cosYaw*dyWorld;
            relativeYaw = atan2( ...
                sin(prediction.hypothesisYaw(step, mode)-ego.yaw), ...
                cos(prediction.hypothesisYaw(step, mode)-ego.yaw));
            deltaX = localX-centerX;
            deltaY = localY-centerY;
            along = cos(relativeYaw)*deltaX + sin(relativeYaw)*deltaY;
            across = -sin(relativeYaw)*deltaX + cos(relativeYaw)*deltaY;
            sigma = prediction.hypothesisSigma(step, mode);
            sigmaAlong = max(sqrt(sigma^2 + ...
                (prediction.length/2)^2), cfg.grid.resolution/2);
            sigmaAcross = max(sqrt(sigma^2 + ...
                (prediction.width/2)^2), cfg.grid.resolution/2);
            footprintProbability = modeProbability*exp(-0.5*( ...
                (along/sigmaAlong).^2 + (across/sigmaAcross).^2));
            riskMap(:, :, step) = 1 - ...
                (1-riskMap(:, :, step)).*(1-footprintProbability);
        end
    end
end
end
