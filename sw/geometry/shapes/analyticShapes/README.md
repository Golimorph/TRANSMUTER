# Analytic shapes

Analytic shapes implement `GeometryIf` directly from compact parameters instead of storing sampled
points. They answer containment and bounds queries efficiently and reject mutating `cut` and
`combine` operations.

`Cylinder` is aligned with local +Z and is used by `Endmill` as the analytic cutting-tool model.
Its renderer samples are delegated to `pointcloudshapes::CylinderGenerator`.

These classes are geometry-module implementation details. Other modules construct them through
`GeometryFactory` and receive `GeometryIf` pointers.
