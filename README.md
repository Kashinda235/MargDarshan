# MargDarshan

MargDarshan is an adaptive pocket-based path planner for autonomous vehicles on
unstructured Indian roads, simulated in MATLAB/Simulink.

This repository contains the MargDarshan project scaffold, default
configuration, shared data contracts, navigation utilities, a lightweight
MATLAB-native simulation world, and Stage 2 simulated-sensor perception.
Perception simulates noisy camera/lidar/radar measurements, associates and
tracks detections, and builds an ego-centric grid map; it does not train or
run real vision models. Stage 3 adds interpretable constant-velocity and
Boid-inspired multi-hypothesis agent predictions, a predictor dispatch
interface, and an ego-frame collision-risk map. Stage 4 implements
time-aware risk fields and connected safe-pocket extraction,
including pocket boundaries, clearances, quality, and a forward corridor.
The local planner samples fixed-capacity Frenet-like trajectories, checks
their kinematic and pocket feasibility, scores static and predicted-agent
risk, and selects a trajectory using an adaptive cruise/cautious/emergency
policy with hysteresis. Run `demoPlanner` to compare candidate paths and the
selected trajectory at frozen snapshots of the village and intersection
scenarios. Vehicle dynamics and the simulation integration loop remain
outside the local planner.

Phase 8 integrates the existing modules in a fixed-step closed loop.
`runSimulation` supports the MargDarshan planner and a constant-time-gap
ACC lane-following baseline; both use the same vehicle model, sensors,
tracking, and collision checks. `computeMetrics` summarizes safety,
completion, timing, and motion smoothness. Run `sim/main.m` for the
five-scenario comparison and a village animation; result tables and video
are written to `/results`. Use `animateRun` to render another logged run.

The world module provides deterministic `village`, `intersection`,
`highwayMerge`, `market`, and `cattleCrossing` scenarios, scripted/reactive
agents, a ground-truth ego-frame grid, collision checks, and plotting. Run
`demoWorld` from MATLAB to animate every scenario for 10 seconds with a
stationary ego.

Run the unit tests from the project root:

```matlab
matlab -batch "addpath(pwd); startup; results = runtests('tests'); assertSuccess(results)"
```

Run the Stage 2 market perception demonstration with `demoPerception` from
MATLAB. It simulates 20 seconds, plots tracked agents against ground truth,
and reports mean position error and ID-switch count.

Run `demoPrediction` from MATLAB to replay the intersection scenario, overlay
constant-velocity and Boid predictions, and report ADE/FDE for both methods.

Run `demoPocket` from MATLAB to animate the safe pocket in `market` and
`cattleCrossing` for 15 seconds per scenario.

See [AGENTS.md](AGENTS.md) for coordinate-frame, units, MATLAB, and phase
conventions.
