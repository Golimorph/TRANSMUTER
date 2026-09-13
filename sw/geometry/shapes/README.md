# Example shapes

These generators create simple sampled geometries for demonstrations and algorithm development.
They currently return `PointCloud` objects, which also satisfy the representation-independent
`GeometryIf` contract.

`Cylinder::generate` samples the boundary. `Cylinder::generateSolid` fills the cylinder volume and
is therefore suitable for material-removal demonstrations. `Box` and `Endmill` currently generate
surface samples.
