# Path generation

`PathGenerator` runs a selected path algorithm in the background. A `MillingScene` supplies
immutable stock, target-model, and tool geometries through `GeometryIf`.

`PathGenerator` owns implementation selection through its `AlgorithmType`. The selected algorithm
receives its immutable execution settings in `PathGeneratorConfig` during construction, including
`geometryTolerance`, the smallest meaningful spatial distance in the simulated geometry.

Generation has two outputs with different publication lifetimes:

- the scene-owned tool path is updated while generation runs so visualization can show it growing;
- the locally simulated workpiece is published in `PathGenerationResult` only when generation
  completes.

This division supports future local operations: a selected region can become a local stock/model
scene, then its generated workpiece can replace that region in the larger scene.
