# Path generation

`PathGenerator` runs a selected algorithm on a worker thread. The program supplies a geometry scene,
machine setup, algorithm settings, and its one stop signal. `PathGenerationContext` owns the
program-wide machine path and immutable machine setup used to create operation planners. Each
generation forks fresh machine kinematics with an empty path, then creates a new algorithm for that
operation. The algorithm returns its own machine path and updated local workpiece in a
`PathGenerationResult`.

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

`PathAlgorithmConfig` holds the geometric settings. Machine sampling and path tolerance are
configured when machine kinematics are created, so algorithms do not carry machine validation
policy or know its axis layout or force model. Each operation has a private workpiece and a fresh
machine-owned path. Later operations get new kinematics and a new path, so they cannot clear or
mutate a completed operation's result. Programs merge completed operation paths into the
program-wide path. See
[machine dynamics](dynamics/README.md) and [the machining model](../../docs/machining-model.md).
