# Changelog

## Simulink integration

- Added fixed-size Simulink bus definitions generated from the constructors in
  `+types`, a reproducible model builder, model initialization, and a
  `runSimulinkScenario` entry point.
- The model's MATLAB System runner calls the existing `runSimulation` pipeline
  in MATLAB during block setup and then emits fixed-size bus samples at the
  configured control rate. The named downstream subsystems currently forward
  that sampled bus; they do not independently execute the corresponding
  algorithms. This compatibility adapter makes the model executable and
  loggable without changing the existing algorithms.
  `adaptiveConfig` continues to provide the existing cruise/cautious/emergency
  hysteresis using its explicit `previousDecision` input.
- The following pipeline functions remain interpreted rather than being
  placed in MATLAB Function blocks: `runSimulation`, `stepWorld`,
  `simulateSensors`, `fuseDetections`, `trackAgents`, `buildGridMap`,
  `predictAgents`, `planLocal`, `trackTrajectory`, `vehicleModel`, and
  `checkCollision`. Their current orchestration uses timing calls, dynamic
  intermediate results, variable-size collections, and/or variable indexing.
  They were not rewritten for code generation because that would expand the
  change into algorithm refactoring and risk altering verified behavior. The
  minimal interface adaptation is the fixed-size contract sample produced by
  `MargDarshanRunner`; the MATLAB algorithms and their state transitions are
  unchanged.
- Added selected-trajectory cost to the MATLAB run log so the Simulink
  workspace and Simulation Data Inspector can record the requested cost signal.
