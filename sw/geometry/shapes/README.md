# Example shapes

These shapes provide simple geometries for demonstrations and algorithm development. `Box` and
the cylinder sampling factories return `PointCloud` objects, which satisfy the
representation-independent `GeometryIf` contract.

`Cylinder::generate` and `Box::generate` sample boundaries. Their `generateSolid` variants fill
the corresponding volume and are suitable for point-cloud solid operations. A constructed
`Cylinder` stores only its radius and height and answers containment analytically. `Endmill` uses
this compact cylinder representation for the tool.
