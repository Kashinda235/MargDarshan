function [agents, state] = trackAgents(detections, state, dt, cfg)
%TRACKAGENTS Update a fixed-capacity constant-velocity Kalman tracker.
% Inputs: detections, fuseDetections output with world-frame positions in m
% and velocities in m/s; state, types.trackerState; dt in s; cfg,
% defaultConfig output.
% Outputs: confirmed and coasting agents in the types.agentList contract,
% plus updated explicit tracker state. Acceleration is m/s^2 and yaw rad.
% Example: [agents, state] = trackAgents(fused, state, 0.1, cfg);

if nargin < 4 || isempty(cfg)
    cfg = defaultConfig();
end
validateattributes(dt, {'numeric'}, ...
    {'scalar','real','finite','positive'}, mfilename, 'dt');
if isempty(state)
    state = types.trackerState(cfg);
end
if detections.count < 0 || detections.count > numel(detections.items)
    error('trackAgents:InvalidDetectionCount', ...
        'detections.count must be within the fixed item capacity.');
end
agents = types.agentList(cfg);

transition = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
accelerationVariance = ...
    cfg.perception.trackerProcessAccelerationStdMps2^2;
processNoise = accelerationVariance * ...
    [dt^4/4 0 dt^3/2 0; ...
     0 dt^4/4 0 dt^3/2; ...
     dt^3/2 0 dt^2 0; ...
     0 dt^3/2 0 dt^2];
activeTracks = find(state.active);
for k = 1:numel(activeTracks)
    index = activeTracks(k);
    state.x(:,index) = transition*state.x(:,index);
    state.covariance(:,:,index) = transition ...
        * state.covariance(:,:,index)*transition' + processNoise;
    state.age(index) = state.age(index)+1;
end

pairCapacity = numel(activeTracks)*detections.count;
pairs = zeros(pairCapacity, 3);
pairCount = 0;
for trackNumber = 1:numel(activeTracks)
    trackIndex = activeTracks(trackNumber);
    for detectionIndex = 1:detections.count
        distance = hypot(state.x(1,trackIndex) ...
            - detections.items(detectionIndex).x, ...
            state.x(2,trackIndex) ...
            - detections.items(detectionIndex).y);
        if distance <= cfg.perception.associationGateM
            pairCount = pairCount+1;
            pairs(pairCount,:) = [distance trackIndex detectionIndex];
        end
    end
end
pairs = sortrows(pairs(1:pairCount,:), 1);
trackUsed = false(1, cfg.maxAgents);
detectionUsed = false(1, detections.count);
for k = 1:pairCount
    trackIndex = pairs(k,2);
    detectionIndex = pairs(k,3);
    if trackUsed(trackIndex) || detectionUsed(detectionIndex)
        continue
    end
    detection = detections.items(detectionIndex);
    previousVelocity = state.x(3:4,trackIndex);
    observation = [1 0 0 0; 0 1 0 0];
    measurement = [detection.x; detection.y];
    variance = max(detection.positionVariance, eps);
    state = kalmanUpdate(state, trackIndex, observation, measurement, ...
        variance*eye(2));
    if detection.hasRadar
        radialObservation = [0 0 detection.radialUnitX ...
            detection.radialUnitY];
        state = kalmanUpdate(state, trackIndex, radialObservation, ...
            detection.radialVelocity, ...
            max(detection.radialVelocityVariance, eps));
    end
    state.acceleration(:,trackIndex) = ...
        (state.x(3:4,trackIndex)-previousVelocity)/dt;
    state.length(trackIndex) = max(detection.length, state.length(trackIndex));
    state.width(trackIndex) = max(detection.width, state.width(trackIndex));
    if detection.hasCamera || detection.hasLidar
        state.yaw(trackIndex) = detection.yaw;
    end
    state.type(trackIndex) = detection.type;
    state.isStatic(trackIndex) = detection.isStatic;
    state.confidence(trackIndex) = 1.0 ...
        -(1.0-state.confidence(trackIndex)) ...
        *(1.0-detection.confidence);
    state.hits(trackIndex) = state.hits(trackIndex)+1;
    state.misses(trackIndex) = 0;
    state.confirmed(trackIndex) = state.confirmed(trackIndex) ...
        || state.hits(trackIndex) >= cfg.perception.trackerConfirmationHits;
    trackUsed(trackIndex) = true;
    detectionUsed(detectionIndex) = true;
end

for k = 1:numel(activeTracks)
    index = activeTracks(k);
    if ~trackUsed(index)
        state.misses(index) = state.misses(index)+1;
        state.confidence(index) = state.confidence(index) ...
            * exp(-dt/cfg.perception.trackerConfidenceDecayTimeS);
        if state.misses(index) > cfg.perception.trackerMaxCoastFrames
            state.active(index) = false;
            state.confirmed(index) = false;
        end
    end
end

for detectionIndex = 1:detections.count
    if ~detectionUsed(detectionIndex)
        state = birthTrack(state, detections.items(detectionIndex), cfg);
    end
end

for k = 1:cfg.maxAgents
    if state.active(k) && state.confirmed(k)
        agent = types.agentState(cfg);
        agent.id = state.id(k);
        agent.type = state.type(k);
        agent.x = state.x(1,k);
        agent.y = state.x(2,k);
        agent.vx = state.x(3,k);
        agent.vy = state.x(4,k);
        agent.ax = state.acceleration(1,k);
        agent.ay = state.acceleration(2,k);
        if hypot(agent.vx, agent.vy) > eps
            agent.yaw = atan2(agent.vy, agent.vx);
        else
            agent.yaw = state.yaw(k);
        end
        agent.length = state.length(k);
        agent.width = state.width(k);
        agent.confidence = state.confidence(k);
        agent.valid = true;
        agents.count = agents.count+1;
        agents.agents(agents.count) = agent;
    end
end
end

function state = kalmanUpdate(state, index, observation, measurement, noise)
covariance = state.covariance(:,:,index);
innovationCovariance = observation*covariance*observation'+noise;
gain = (covariance*observation')/innovationCovariance;
innovation = measurement-observation*state.x(:,index);
state.x(:,index) = state.x(:,index)+gain*innovation;
identity = eye(4);
state.covariance(:,:,index) = (identity-gain*observation) ...
    * covariance * (identity-gain*observation)' ...
    + gain*noise*gain';
end

function state = birthTrack(state, detection, cfg)
index = find(~state.active, 1);
if isempty(index)
    return
end
state.active(index) = true;
state.confirmed(index) = ...
    cfg.perception.trackerConfirmationHits <= 1;
state.id(index) = state.nextId;
state.nextId = state.nextId+1;
state.hits(index) = 1;
state.misses(index) = 0;
state.age(index) = 1;
state.x(:,index) = [detection.x; detection.y; 0; 0];
if detection.hasRadar
    state.x(3,index) = detection.radialVelocity ...
        * detection.radialUnitX;
    state.x(4,index) = detection.radialVelocity ...
        * detection.radialUnitY;
end
state.covariance(:,:,index) = diag([ ...
    cfg.perception.trackerInitialPositionStdM^2, ...
    cfg.perception.trackerInitialPositionStdM^2, ...
    cfg.perception.trackerInitialVelocityStdMps^2, ...
    cfg.perception.trackerInitialVelocityStdMps^2]);
state.acceleration(:,index) = [0; 0];
state.length(index) = detection.length;
state.width(index) = detection.width;
state.yaw(index) = detection.yaw;
state.type(index) = detection.type;
state.confidence(index) = detection.confidence;
state.isStatic(index) = detection.isStatic;
end
