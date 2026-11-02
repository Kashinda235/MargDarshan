function [agentList, staticObstacles, scenario] = stepWorld( ...
        scenario, agentList, ego, dt, cfg)
%STEPWORLD Advance ground-truth agents by one time step.
% Inputs: scenario, world and behavior state; agentList, fixed agent array;
% ego, ego pose/speed; dt in s; cfg, defaultConfig output.
% Outputs: updated agentList, fixed static obstacle list, and scenario
% with elapsed time and reactive behavior state updated.
% Example: [agents,obs,s] = stepWorld(s,s.agents,s.egoStart,0.1,cfg);

if nargin < 5 || isempty(cfg)
    cfg = defaultConfig();
end
validateattributes(dt, {'numeric'}, ...
    {'scalar','real','finite','positive'}, mfilename, 'dt');
if agentList.count < 0 || agentList.count > cfg.maxAgents
    error('stepWorld:InvalidAgentCount', ...
        'agentList.count must be between 0 and cfg.maxAgents.');
end
[desiredVelocity, scenario] = agentBehaviors( ...
    scenario, agentList, ego, dt, cfg);
for k = 1:agentList.count
    agent = agentList.agents(k);
    targetVelocity = desiredVelocity(k, :);
    targetSpeed = hypot(targetVelocity(1), targetVelocity(2));
    speedLimit = cfg.world.maxSpeeds(agent.type);
    if targetSpeed > speedLimit
        targetVelocity = targetVelocity * speedLimit/targetSpeed;
    end
    currentVelocity = [agent.vx agent.vy];
    deltaVelocity = targetVelocity - currentVelocity;
    deltaSpeed = hypot(deltaVelocity(1), deltaVelocity(2));
    accelerationLimit = cfg.world.maxAccelerations(agent.type);
    decelerationLimit = cfg.world.maxDecelerations(agent.type);
    if deltaSpeed > 0
        if targetSpeed < hypot(currentVelocity(1), currentVelocity(2))
            maxDelta = decelerationLimit * dt;
        else
            maxDelta = accelerationLimit * dt;
        end
        if deltaSpeed > maxDelta
            deltaVelocity = deltaVelocity * maxDelta/deltaSpeed;
        end
    end
    nextVelocity = currentVelocity + deltaVelocity;
    nextSpeed = hypot(nextVelocity(1), nextVelocity(2));
    if nextSpeed > speedLimit
        nextVelocity = nextVelocity * speedLimit/nextSpeed;
    end
    agent.ax = (nextVelocity(1)-agent.vx)/dt;
    agent.ay = (nextVelocity(2)-agent.vy)/dt;
    agent.vx = nextVelocity(1);
    agent.vy = nextVelocity(2);
    agent.x = agent.x + agent.vx*dt;
    agent.y = agent.y + agent.vy*dt;
    if hypot(agent.vx, agent.vy) > 0
        agent.yaw = atan2(agent.vy, agent.vx);
    end
    agentList.agents(k) = agent;
end
scenario.time = scenario.time + dt;
scenario.stepIndex = scenario.stepIndex + 1;
staticObstacles = scenario.obstacles;
end
