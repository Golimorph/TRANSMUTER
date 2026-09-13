# Geometry

The geometry module owns shape representations and operations on them. Code outside this module
works with `GeometryIf` instead of assuming that a solid is stored as a point cloud.

Geometry implementations enforce their invariants when they are constructed. Once a geometry is
published through `GeometryIf`, clients can treat it as valid and use only the
representation-independent queries they need:

- axis-aligned bounds;
- maximum distance from an arbitrary axis;
- independent cloning;
- a sampled view for the current renderer.

`PointCloud` is the first implementation. It remains useful for developing and visualizing milling
algorithms, but it is not part of the solid-geometry contract used by path algorithms.

Material removal and collision logic also live here:

- `MaterialRemovalIf` subtracts a tool sweep from a mutable geometry;
- `CollisionDetectorIf` performs general transformed-geometry collision queries;
- `VerticalToolClearanceIf` lifts a supplied vertical tool pose only when it would intersect an
  obstacle. It owns the representation-specific clearance calculation; path algorithms do not
  inspect tool points or pass point-cloud dimensions into the query.

Factory functions select the compatible backend. Today they select point-cloud implementations.
A future mesh, voxel grid, spatially indexed cloud, or signed-distance field can add corresponding
implementations without changing path-generation or visualization orchestration.
