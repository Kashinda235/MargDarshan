function [desiredVelocity, scenario] = agentBehaviors(scenario, agentList, ...
        ego, dt, cfg)
%AGENTBEHAVIORS Evaluate deterministic scripted/reactive ground-truth motion.
% Inputs: scenario and agentList, world-frame agent records; ego, ego pose
% and speed; dt in s; cfg, defaultConfig output.
% Outputs: desiredVelocity, [cfg.maxAgents x 2] world-frame m/s; scenario
% with behavior trigger/goal state updated.
% Example: [v,s] = agentBehaviors(s, s.agents, s.egoStart, 0.1, cfg);

validateattributes(dt, {'numeric'}, {'scalar','real','finite','positive'});
desiredVelocity = zeros(cfg.maxAgents, 2);
frame = scenario.stepIndex + 1;
streamSeed = mod(scenario.seed + frame, 2^32 - 1);
stream = RandStream('mt19937ar', 'Seed', streamSeed);
for k = 1:agentList.count
    agent = agentList.agents(k);
    behavior = scenario.behaviors(k);
    switch behavior.model
        case cfg.world.behaviorModels.stationary
            velocity = [0 0];
        case cfg.world.behaviorModels.laneFollow
            lateralTarget = behavior.laneY + behavior.jitter * ...
                sin(2*pi*scenario.time/cfg.world.laneJitterPeriod + ...
                behavior.phase);
            lateralSpeed = cfg.world.laneFollowGain * ...
                (lateralTarget - agent.y);
            velocity = [behavior.direction*behavior.speed, lateralSpeed];
        case cfg.world.behaviorModels.straight
            velocity = behavior.speed * [cos(agent.yaw) sin(agent.yaw)];
        case cfg.world.behaviorModels.pedestrian
            targetDelta = [behavior.targetX-agent.x, ...
                behavior.targetY-agent.y];
            targetDistance = hypot(targetDelta(1), targetDelta(2));
            if targetDistance <= cfg.world.pedestrianGoalRadius
                if behavior.goalChanges < cfg.world.maxBehaviorGoalChanges
                    angle = 2*pi*rand(stream, 1, 1);
                    distance = cfg.world.pedestrianGoalRadius + ...
                        rand(stream, 1, 1) * ...
                        (cfg.world.pedestrianGoalRadiusMax - ...
                        cfg.world.pedestrianGoalRadius);
                    behavior.targetX = agent.x + distance*cos(angle);
                    behavior.targetY = agent.y + distance*sin(angle);
                    behavior.goalChanges = behavior.goalChanges + 1;
                    scenario.behaviors(k) = behavior;
                    targetDelta = [behavior.targetX-agent.x, ...
                        behavior.targetY-agent.y];
                    targetDistance = hypot(targetDelta(1), targetDelta(2));
                else
                    targetDistance = 0;
                end
            end
            if targetDistance > 0
                velocity = behavior.speed * targetDelta / targetDistance;
            else
                velocity = [0 0];
            end
        case cfg.world.behaviorModels.cattleCrossing
            if ~behavior.triggered && hypot(agent.x-ego.x, agent.y-ego.y) ...
                    <= behavior.triggerDistance
                behavior.triggered = true;
                scenario.behaviors(k) = behavior;
            end
            if behavior.triggered
                velocity = [0 behavior.crossingDirection * behavior.speed];
            else
                velocity = [0 0];
            end
        case cfg.world.behaviorModels.merge
            lateralSpeed = cfg.world.mergeGain * ...
                (behavior.mergeY - agent.y);
            velocity = [behavior.speed, lateralSpeed];
        otherwise
            error('agentBehaviors:UnknownModel', ...
                'Unsupported behavior model %d for agent %d.', ...
                behavior.model, agent.id);
    end
    desiredVelocity(k, :) = velocity;
end
end
