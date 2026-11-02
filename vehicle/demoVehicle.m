function metrics = demoVehicle(cfg)
%DEMOVEHICLE Simulate a double-lane change with a stop-and-go speed profile.
% Inputs: cfg, defaultConfig output; vehicle and controller parameters use
% SI units and cfg.simDt (s).
% Outputs: metrics with simulated state history, tracking errors, and figure.
% Example: result = demoVehicle(defaultConfig());

if nargin < 1 || isempty(cfg)
    cfg = defaultConfig();
end
trajectory = makeDemoTrajectory(cfg);
state = types.egoState();
controllerState = [];
stepCount = numel(trajectory.t);
stateHistory = repmat(state,stepCount,1);
errorHistory = zeros(stepCount,3);

for k = 1:stepCount
    stateHistory(k) = state;
    controlTrajectory = trajectory;
    controlTrajectory.v(:) = trajectory.v(k);
    controlTrajectory.a(:) = trajectory.a(k);
    errors = controlDiagnostics(state,controlTrajectory);
    errorHistory(k,:) = [errors.crossTrackError,errors.headingError, ...
        errors.speedError];
    if k < stepCount
        [command,controllerState] = trackTrajectory( ...
            state,controlTrajectory,controllerState,cfg);
        state = vehicleModel(state,command,cfg);
    end
end

fig = figure('Name','Stage 5 vehicle tracking');
subplot(2,1,1);
plot(trajectory.x,trajectory.y,'k--','LineWidth',1.2);
hold on;
plot(reshape([stateHistory.x],[],1),reshape([stateHistory.y],[],1), ...
    'b-','LineWidth',1.2);
axis equal;
grid on;
xlabel('East x (m)');
ylabel('North y (m)');
legend('Reference','Vehicle','Location','best');
title('Double-lane-change tracking');
subplot(2,1,2);
plot(trajectory.t,errorHistory(:,1),'LineWidth',1.1);
hold on;
plot(trajectory.t,errorHistory(:,2),'LineWidth',1.1);
plot(trajectory.t,errorHistory(:,3),'LineWidth',1.1);
grid on;
xlabel('Time (s)');
ylabel('Tracking error (m, rad, m/s)');
legend('Cross-track (m)','Heading (rad)','Speed (m/s)', ...
    'Location','best');
title('Tracking errors');

metrics = struct( ...
    'trajectory',trajectory, ...
    'stateHistory',stateHistory, ...
    'trackingErrors',errorHistory, ...
    'maxAbsCrossTrackErrorM',max(abs(errorHistory(:,1))), ...
    'maxAbsHeadingErrorRad',max(abs(errorHistory(:,2))), ...
    'maxAbsSpeedErrorMps',max(abs(errorHistory(:,3))), ...
    'figure',fig);
end

function trajectory = makeDemoTrajectory(cfg)
t = (0:cfg.simDt:cfg.vehicleDemo.durationS)';
if t(end) < cfg.vehicleDemo.durationS
    t(end+1,1) = cfg.vehicleDemo.durationS;
end
speed = interp1(cfg.vehicleDemo.speedTimesS, ...
    cfg.vehicleDemo.speedValuesMps,t,'linear');
speed = max(0,speed);
acceleration = gradient(speed,t);
x = cumtrapz(t,speed);
distanceBounds = cfg.vehicleDemo.laneChangeDistanceM;
offsets = cfg.vehicleDemo.laneChangeOffsetsM;
y = offsets(1)*smoothStep(x,distanceBounds(1),distanceBounds(2)) ...
    +offsets(2)*smoothStep(x,distanceBounds(3),distanceBounds(4)) ...
    +offsets(3)*smoothStep(x,distanceBounds(5),distanceBounds(6));
dx = gradient(x,t);
dy = gradient(y,t);
yaw = zeros(size(t));
moving = hypot(dx,dy) > eps;
yaw(moving) = atan2(dy(moving),dx(moving));
for k = 2:numel(yaw)
    if ~moving(k)
        yaw(k) = yaw(k-1);
    end
end
pathSpeed = hypot(dx,dy);
kappa = gradient(unwrap(yaw),t)./max(pathSpeed,eps);
kappa(~isfinite(kappa)) = 0.0;
trajectory = struct( ...
    't',t, ...
    'x',x, ...
    'y',y, ...
    'yaw',yaw, ...
    'kappa',kappa, ...
    'v',speed, ...
    'a',acceleration);
end

function value = smoothStep(position,startPosition,endPosition)
fraction = min(1.0,max(0.0, ...
    (position-startPosition)/(endPosition-startPosition)));
value = 10*fraction.^3-15*fraction.^4+6*fraction.^5;
end
