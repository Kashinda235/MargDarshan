function tests = testVehicle()
%TESTVEHICLE Unit tests for Stage 5 vehicle dynamics and controllers.
tests = functiontests(localfunctions);
end

function testSpeedStepSettlesWithoutExcessOvershoot(testCase)
cfg = defaultConfig();
path = straightTrajectory(80.0,cfg.simDt,5.0);
state = types.egoState();
controllerState = [];
speed = zeros(numel(path.t),1);
for k = 1:numel(path.t)
    speed(k) = state.v;
    [command,controllerState] = trackTrajectory( ...
        state,path,controllerState,cfg);
    state = vehicleModel(state,command,cfg);
end
steady = speed(end-round(2/cfg.simDt):end);
verifyLessThanOrEqual(testCase,max(speed)-5.0,0.5);
verifyLessThanOrEqual(testCase,abs(mean(steady)-5.0),0.15);
end

function testTracksConstantRadiusCircle(testCase)
cfg = defaultConfig();
radius = 15.0;
speed = 4.0;
spacing = cfg.simDt*speed;
angle = (0:spacing/radius:1.2*pi)';
path = struct( ...
    't',angle*radius/speed, ...
    'x',radius*cos(angle), ...
    'y',radius*sin(angle), ...
    'yaw',angle+pi/2, ...
    'kappa',ones(size(angle))/radius, ...
    'v',speed*ones(size(angle)), ...
    'a',zeros(size(angle)));
state = types.egoState();
state.x = radius;
state.yaw = pi/2;
state.v = speed;
controllerState = [];
errors = zeros(round(15/cfg.simDt),1);
for k = 1:numel(errors)
    [command,controllerState] = trackTrajectory( ...
        state,path,controllerState,cfg);
    state = vehicleModel(state,command,cfg);
    diagnostics = controlDiagnostics(state,path);
    errors(k) = diagnostics.crossTrackError;
end
verifyLessThan(testCase,max(abs(errors)),1.0);
end

function testSteeringRateAndAccelerationLimits(testCase)
cfg = defaultConfig();
state = types.egoState();
command = struct('steer',cfg.vehicle.maxSteer*2, ...
    'accel',cfg.vehicle.maxAccel*2);
previousSteer = state.steer;
for k = 1:round(3/cfg.simDt)
    state = vehicleModel(state,command,cfg);
    verifyLessThanOrEqual(testCase, ...
        abs(state.steer-previousSteer)/cfg.simDt, ...
        cfg.vehicle.maxSteerRate+1e-8);
    verifyLessThanOrEqual(testCase,state.steer,cfg.vehicle.maxSteer);
    verifyLessThanOrEqual(testCase,state.a,cfg.vehicle.maxAccel);
    verifyGreaterThanOrEqual(testCase,state.a,-cfg.vehicle.maxDecel);
    previousSteer = state.steer;
end
end

function testControllerStatePersistsBetweenCalls(testCase)
cfg = defaultConfig();
cfg.controller.speedKp = 0.2;
path = straightTrajectory(10.0,cfg.simDt,4.0);
state = types.egoState();
[~,firstState] = longitudinalController(state,path,[],cfg);
[~,secondState] = longitudinalController(state,path,firstState,cfg);
verifyTrue(testCase,secondState.initialized);
verifyGreaterThan(testCase,secondState.speedIntegral, ...
    firstState.speedIntegral);
verifyEqual(testCase,secondState.previousSpeedError, ...
    firstState.previousSpeedError);
end

function testDynamicModelAndControllerDiagnostics(testCase)
cfg = defaultConfig();
cfg.vehicleModel = 'dynamic';
state = types.egoState();
state.v = 5.0;
command = struct('steer',0.1,'accel',0.0);
next = vehicleModel(state,command,cfg);
verifyTrue(testCase,isfinite(next.yawRate));
verifyTrue(testCase,isfinite(next.vy));

path = straightTrajectory(10.0,cfg.simDt,2.0);
state.y = 1.0;
state.v = 0.0;
diagnostics = controlDiagnostics(state,path);
verifyGreaterThan(testCase,diagnostics.crossTrackError,0);
verifyEqual(testCase,diagnostics.speedError,2.0);
end

function testVehicleDemoTracksLaneChangeAndStopGo(testCase)
cfg = defaultConfig();
cfg.vehicleDemo.durationS = 12.0;
cfg.vehicleDemo.speedTimesS = [0 2 6 8 10 12];
cfg.vehicleDemo.speedValuesMps = [0 5 5 0 0 5];
cfg.vehicleDemo.laneChangeDistanceM = [2 8 10 18 20 30];
visibility = get(groot,'defaultFigureVisible');
cleanupVisibility = onCleanup(@() ...
    set(groot,'defaultFigureVisible',visibility));
set(groot,'defaultFigureVisible','off');
result = demoVehicle(cfg);
cleanupFigure = onCleanup(@() closeIfValid(result.figure));
verifyEqual(testCase,size(result.trackingErrors,2),3);
verifyTrue(testCase,all(isfinite(result.trackingErrors(:))));
verifyGreaterThan(testCase,max(result.trajectory.y),0);
verifyLessThan(testCase,min(result.trajectory.y),0);
clear cleanupFigure cleanupVisibility
end

function path = straightTrajectory(distance,dt,targetSpeed)
t = (0:dt:distance/targetSpeed)';
path = struct( ...
    't',t, ...
    'x',targetSpeed*t, ...
    'y',zeros(size(t)), ...
    'yaw',zeros(size(t)), ...
    'kappa',zeros(size(t)), ...
    'v',targetSpeed*ones(size(t)), ...
    'a',zeros(size(t)));
end

function closeIfValid(handle)
if isgraphics(handle)
    close(handle);
end
end
