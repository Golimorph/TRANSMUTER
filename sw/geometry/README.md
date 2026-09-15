# Geometry

The geometry module owns shape representations, shape construction, and operations on them. Code
outside this module includes only headers from `if` and works with `GeometryIf` rather than
assuming that a solid is stored as a point cloud.

Geometry implementations enforce their invariants when they are constructed. Once a geometry is
published through `GeometryIf`, clients can treat it as valid and use only the
representation-independent queries they need:

- point containment;
- transformed solid intersection;
- combining and cutting solids at a supplied transform;
- axis-aligned bounds;
- maximum distance from an arbitrary axis;
- independent cloning;
- a sampled view for the current renderer.

`PointCloud` is the first mutable implementation. Its `cut` operation removes each sampled point
which the transformed cutting geometry reports as inside. A batch overload applies a tool sweep in
one spatially indexed pass. Its approximate `isInside` query treats the cloud's minimum
point-to-point spacing as the sampling tolerance.

The `if` directory is the module's public boundary. It contains `GeometryIf`, `ToolPathIf`, shared
value types, `MillingScene`, and `GeometryFactory`. The factory provides typed creation methods and
returns interface pointers, so callers cannot depend on concrete representations.

`PointCloud` is private implementation storage under `pointCloud`. Concrete shape construction is
private under `shapes` and divided by representation:

- `shapes/analyticShapes` contains immutable parameterized geometry such as the end-mill cylinder.
- `shapes/pointCloudShapes` contains generators for sampled boxes, cylinders, dodecahedra, and tori.

Analytic tools derive from `AnalyticShape` and answer containment directly from their parameters.
Point-cloud generators keep sampling concerns outside those analytic implementations.
