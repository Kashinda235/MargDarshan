# MargDarshan project conventions

## Scope
- This repository is the scaffold and shared data contracts for MargDarshan, an adaptive pocket-based path planner for autonomous vehicles on unstructured Indian roads.
- Do not add planning, perception, prediction, vehicle-control, or simulation algorithms until their designated phase.

## Coordinate frames and units
- World frame: ENU; x points east, y points north, and yaw is in radians counter-clockwise from +x.
- Local/grid frame: ego-centric; x points forward and y points left.
- Use SI units everywhere. State units in function headers and contract documentation.

## MATLAB implementation constraints
- Every module must be a pure function or a small class with no hidden global state, so it can later be called from a Simulink MATLAB Function block.
- Avoid dynamic struct fields and mixed-type cell arrays. Preallocate arrays and use fixed maximum sizes such as `cfg.maxAgents`.
- Keep every tunable parameter in `config/defaultConfig.m`; module code must not contain magic numbers.
- Each function must have a header comment documenting inputs, outputs, units, and an example.
- Every module must ship with at least one unit test in `/tests`, using `matlab.unittest`.
- Use `startup` to add project folders to the MATLAB path.

## Data contracts
- Constructors in `+types` return fully initialised structs with fixed field sizes.
- Agent type integer values are: 1 car, 2 bus, 3 truck, 4 tractor, 5 auto-rickshaw, 6 motorcycle, 7 bicycle, 8 pedestrian, 9 pushcart, and 10 cattle.
- `agentList` is fixed-capacity and uses `cfg.maxAgents`.
- Grid layers use `[ny x nx]` dimensions; probability and traversability values are in `[0,1]`.
- Predicted-agent arrays use the configured prediction step count and fixed mode count.
- Pocket boundaries use a fixed maximum vertex count; `boundaryCount` indicates populated vertices.
- Trajectory samples use the configured planner sample count.
- Decision mode values are: 1 cruise, 2 cautious, and 3 emergency.

## Phase folders
- `/nav`: Phase 1; `/world`: Phase 2; `/perception`: Phase 3.
- `/prediction`: Phase 4; `/planning`: Phases 5-6; `/vehicle`: Phase 7.
- `/sim`: Phase 8; `/simulink`: Phase 9.
- `/metrics` and `/tests` hold metrics and verification. `/results` is generated output and is gitignored.

## Test command
```matlab
matlab -batch "addpath(pwd); startup; results = runtests('tests'); assertSuccess(results)"
```
