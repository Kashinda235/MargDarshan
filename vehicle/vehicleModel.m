function nextState = vehicleModel(state, command, cfg)
%VEHICLEMODEL Advance the configured vehicle model by one simulation step.
% Inputs: state has x/y (m), yaw/steer (rad), v/vy (m/s), a (m/s^2),
% and yawRate (rad/s); command has steer (rad) and accel (m/s^2); cfg
% contains vehicle parameters and cfg.simDt (s).
% Outputs: nextState after one RK4 step, with actuator limits and lag.
% Example: state = vehicleModel(state, struct('steer',0,'accel',1), cfg);

if nargin < 3 || isempty(cfg)
    cfg = defaultConfig();
end
validateState(state);
if ~isstruct(command) || ~all(isfield(command, {'steer','accel'})) || ...
        any(~isfinite([command.steer command.accel]))
    error('vehicleModel:InvalidCommand', ...
        'command must contain finite steer (rad) and accel (m/s^2).');
end
if ~isfield(cfg, 'simDt') || ~isfinite(cfg.simDt) || cfg.simDt <= 0
    error('vehicleModel:InvalidTimeStep', 'cfg.simDt must be positive.');
end
if ~isfield(cfg, 'vehicleModel') || ...
        ~any(strcmp(cfg.vehicleModel, {'kinematic','dynamic'}))
    error('vehicleModel:InvalidModel', ...
        'cfg.vehicleModel must be kinematic or dynamic.');
end

v = [state.x; state.y; state.yaw; max(0,state.v); state.steer; ...
    state.a; state.vy; state.yawRate];
steerCommand = min(cfg.vehicle.maxSteer, ...
    max(-cfg.vehicle.maxSteer, command.steer));
accelCommand = min(cfg.vehicle.maxAccel, ...
    max(-cfg.vehicle.maxDecel, command.accel));
dt = cfg.simDt;
k1 = stateDerivative(v, steerCommand, accelCommand, cfg);
k2 = stateDerivative(v+0.5*dt*k1, steerCommand, accelCommand, cfg);
k3 = stateDerivative(v+0.5*dt*k2, steerCommand, accelCommand, cfg);
k4 = stateDerivative(v+dt*k3, steerCommand, accelCommand, cfg);
v = v+(dt/6)*(k1+2*k2+2*k3+k4);
v(4) = max(0, v(4));
v(5) = min(cfg.vehicle.maxSteer, max(-cfg.vehicle.maxSteer, v(5)));
v(6) = min(cfg.vehicle.maxAccel, max(-cfg.vehicle.maxDecel, v(6)));

nextState = state;
nextState.x = v(1);
nextState.y = v(2);
nextState.yaw = atan2(sin(v(3)), cos(v(3)));
nextState.v = v(4);
nextState.steer = v(5);
nextState.a = v(6);
nextState.vy = v(7);
nextState.yawRate = v(8);
if strcmp(cfg.vehicleModel, 'kinematic')
    nextState.vy = 0.0;
    nextState.yawRate = nextState.v/cfg.vehicle.wheelbase ...
        *tan(nextState.steer);
end
end

function derivative = stateDerivative(value, steerCommand, accelCommand, cfg)
v = max(0, value(4));
steerRate = (steerCommand-value(5))/cfg.vehicle.steerLagS;
steerRate = min(cfg.vehicle.maxSteerRate, ...
    max(-cfg.vehicle.maxSteerRate, steerRate));
accelRate = (accelCommand-value(6))/cfg.vehicle.accelLagS;
accel = min(cfg.vehicle.maxAccel, ...
    max(-cfg.vehicle.maxDecel, value(6)));
if v <= 0 && accel < 0
    accel = 0;
end

if strcmp(cfg.vehicleModel, 'dynamic') && ...
        v >= cfg.vehicle.dynamicMinSpeedMps
    vy = value(7);
    yawRate = value(8);
    front = cfg.vehicle.frontAxleM;
    rear = cfg.vehicle.rearAxleM;
    cf = cfg.vehicle.frontCorneringStiffnessNPerRad;
    cr = cfg.vehicle.rearCorneringStiffnessNPerRad;
    mass = cfg.vehicle.massKg;
    inertia = cfg.vehicle.yawInertiaKgM2;
    lateralAccel = -(cf+cr)*vy/(mass*v) ...
        +(rear*cr-front*cf)*yawRate/(mass*v) ...
        -v*yawRate+cf*value(5)/mass;
    yawAccel = (rear*cr-front*cf)*vy/(inertia*v) ...
        -(front^2*cf+rear^2*cr)*yawRate/(inertia*v) ...
        +front*cf*value(5)/inertia;
    xRate = v*cos(value(3))-vy*sin(value(3));
    yRate = v*sin(value(3))+vy*cos(value(3));
    yawDerivative = yawRate;
    vyDerivative = lateralAccel;
    yawRateDerivative = yawAccel;
else
    yawRate = v/cfg.vehicle.wheelbase*tan(value(5));
    xRate = v*cos(value(3));
    yRate = v*sin(value(3));
    yawDerivative = yawRate;
    vyDerivative = -value(7)/cfg.vehicle.dynamicLowSpeedRelaxationS;
    yawRateDerivative = (yawRate-value(8)) ...
        /cfg.vehicle.dynamicLowSpeedRelaxationS;
end

derivative = [xRate; yRate; yawDerivative; accel; steerRate; ...
    accelRate; vyDerivative; yawRateDerivative];
end

function validateState(state)
required = {'x','y','yaw','v','steer','a','vy','yawRate'};
if ~isstruct(state) || ~all(isfield(state, required))
    error('vehicleModel:InvalidState', ...
        'state must contain x, y, yaw, v, steer, a, vy, and yawRate.');
end
if any(~isfinite([state.x state.y state.yaw state.v state.steer ...
        state.a state.vy state.yawRate])) || state.v < 0
    error('vehicleModel:InvalidState', ...
        'State values must be finite and speed must be nonnegative.');
end
end
