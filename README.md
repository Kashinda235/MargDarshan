# MargDarshan

MargDarshan is an adaptive pocket-based path planner for autonomous vehicles on
unstructured Indian roads, simulated in MATLAB/Simulink.

This repository contains the MargDarshan project scaffold, default
configuration, shared data contracts, navigation utilities, and a lightweight
MATLAB-native simulation world. Planning, perception, prediction, and vehicle
control algorithms are still outside the implemented scope.

The world module provides deterministic `village`, `intersection`,
`highwayMerge`, `market`, and `cattleCrossing` scenarios, scripted/reactive
agents, a ground-truth ego-frame grid, collision checks, and plotting. Run
`demoWorld` from MATLAB to animate every scenario for 10 seconds with a
stationary ego.

Run the unit tests from the project root:

```matlab
matlab -batch "addpath(pwd); startup; results = runtests('tests'); assertSuccess(results)"
```

See [AGENTS.md](AGENTS.md) for coordinate-frame, units, MATLAB, and phase
conventions.
