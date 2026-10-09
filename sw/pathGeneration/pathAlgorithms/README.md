# Path algorithms

Algorithms implement `PathAlgorithmIf`. They receive the input scene through `GeometryIf` and a
private workpiece cloned from stock; only the machine implementation chooses joint positions and
checks forces. Each `generate` method prepares a cutting operation, traverses its layers or
surfaces, and returns the resulting workpiece and machine path.

## Raster

Raster follows HEAD's layers, rows, and unidirectional +X cuts. Its cutting operation lifts a
requested cutter pose when the tool intersects the target through `GeometryIf::intersects`, then
asks the machine to accept the move. The machine records accepted endpoints; the raster then
removes cutter material from its private workpiece.

## Spiral

`ThreeAxisSpiralAlgorithm` follows layers and counter-clockwise concentric circles. It uses the
same target-lifting rule and cuts its own private workpiece after accepted moves.

At the move boundary, each cutting operation asks `MachineKinematicsIf::propose` to accept and record one pose.
The algorithm then cuts the private workpiece. A machine
rejection calls `ERROR`.
The one shared stop signal handles cancellation.
The machine checks sampled points between poses; the geometric strategies do not contain machine-axis
or cutting-force rules. Changing row direction after a rejected move is a possible future strategy.

## Experimental normal-surface machining

`FiveAxisNormalAlgorithm` peels exposed surfaces until its pass limit, a stop request, or lack of
progress. `NormalSurfaceCuttingOperation` samples remaining stock and proposes complete candidate
curves along a normal or alternate access axis. It tests each curve on a private workpiece clone
and an independent copy of the machine planner, publishing and cutting only if every point is accepted and material
is removed. Rejection lets the operation try another direction or orientation.

Its `geometry` folder contains `ExposedSurface` and `SurfaceCutGeometry`. The former samples occupied
material and estimates normals; the latter constructs cuts and spherical clearance links and checks
cutting endpoints against the target. Geometry construction does not mutate stock or choose machine
axis solutions. The search reports unproven coverage and unresolved surface candidates.

## Boundaries and generality

Torus dimensions, cutter dimensions and coarse-preview spacing belong to the
example programs. The raster's row construction and target query consume arbitrary stock/target
geometry through interfaces; they contain no torus equations or shape-name dispatch. Their +Z access
and X-row assumptions define a three-axis raster strategy, not a general five-axis solver. Surface
peeling is a separate policy for more general orientation searches.

`dynamics` owns axis search, force estimation, and preload checks behind `MachineKinematicsIf`.
Algorithms own stock contact, target and shank collision checks, and material removal.

All checks remain sampled. Geometry thinner than the chosen spacing may require finer sampling, and
changing representations or force models can change acceptance. The experimental five-axis orientation search tries each
candidate curve against a forked machine planner and cloned workpiece state.
