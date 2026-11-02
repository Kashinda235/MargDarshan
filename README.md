# MargDarshan

MargDarshan is an adaptive pocket-based path planner for autonomous vehicles on
unstructured Indian roads, simulated in MATLAB/Simulink.

This repository contains the MargDarshan project scaffold, default
configuration, shared data contracts, navigation utilities, a lightweight
MATLAB-native simulation world, and Stage 2 simulated-sensor perception.
Perception simulates noisy camera/lidar/radar measurements, associates and
tracks detections, and builds an ego-centric grid map; it does not train or
run real vision models. Prediction, planning, and vehicle control remain
outside the implemented scope.

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

See [AGENTS.md](AGENTS.md) for coordinate-frame, units, MATLAB, and phase
conventions.
