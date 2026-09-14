# Geometry

The geometry module owns shape representations and operations on them. Code outside this module
works with `GeometryIf` instead of assuming that a solid is stored as a point cloud.

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

Analytic tools derive from `AnalyticShape`. They answer containment queries directly from their small
parameter set and deliberately reject `cut` and `combine`, because these immutable shapes are used
as operands rather than mutable workpieces. `Cylinder` is the first analytic shape and is used for
the end mill without first expanding the tool into a point cloud.
