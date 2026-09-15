# Point-cloud shape generators

These stateless factories create sampled `PointCloud` geometries for demonstrations, visualization,
and material-removal simulation:

- `BoxGenerator` and `CylinderGenerator` provide both boundary and solid samples.
- `DodecahedronGenerator` provides pentagonal-face surface samples and a convex solid variant.
- `TorusGenerator` provides smooth surface samples and a non-convex solid variant.

The `generate` methods sample a boundary for visualization. The `generateSolid` methods sample the
corresponding volume and are suitable for point-cloud solid operations.

These generators are geometry-module implementation details. Other modules select the requested
shape and sampling mode through `GeometryFactory` and receive `GeometryIf` pointers.
