# Path generation

`PathGenerator` runs a selected algorithm on a worker thread. The program supplies the scene,
settings, and its one stop signal. The algorithm returns a `PathGenerationResult` containing the
workpiece and machine path.

Each algorithm clones its input stock into a private workpiece. It chooses a geometric move, asks
`MachineKinematicsIf::propose` for a backlash-safe axis endpoint. The machine records accepted
moves; only then does the algorithm cut its private workpiece. The machine reads accepted machine history
to reconstruct preload; it never sees or changes stock, target, or workpiece geometry. Raster and
spiral call `ERROR` on rejection. The experimental five-axis algorithm tries another candidate
without publishing a rejected trial.

| Folder | Responsibility | Public boundary |
| --- | --- | --- |
| `pathAlgorithms` | Construct raster, spiral or surface cuts from `GeometryIf` | `PathAlgorithmIf` and factory |
| `dynamics` | Resolve axes and check motion and preload | `MachineKinematicsIf`, `MachinePathIf` |

`PathAlgorithmConfig` holds the geometric settings. Machine sampling and path
tolerance are configured when the scene's machine is created, so algorithms do not carry machine
validation policy or know its axis layout or force model.
Each roughing or finishing operation has its own private workpiece and machine-owned live path. The generator snapshots each completed path so later stages cannot change its result. The quick program
joins completed +Z operations above the current workpiece. The transfer is checked against the
combined machine history and remaining material before its points are appended. Individually
planned stages still need review as one complete program. See
[machine dynamics](dynamics/README.md) and [the machining model](../../docs/machining-model.md).
