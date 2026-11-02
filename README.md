# MargDarshan

MargDarshan is an adaptive pocket-based path planner for autonomous vehicles on
unstructured Indian roads, simulated in MATLAB/Simulink.

This repository currently contains the project scaffold, default configuration,
shared data-contract constructors, and unit tests. Algorithms are intentionally
not implemented in this scaffold phase.

Run the unit tests from the project root:

```matlab
matlab -batch "addpath(pwd); startup; results = runtests('tests'); assertSuccess(results)"
```

See [AGENTS.md](AGENTS.md) for coordinate-frame, units, MATLAB, and phase
conventions.
