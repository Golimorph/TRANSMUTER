# Shape implementations

This directory contains geometry construction details. Code outside the geometry module must not
include these headers; it creates shapes through `GeometryFactory` and retains them through
`GeometryIf`.

- `analyticShapes` contains compact parameterized shapes used for fast geometric queries.
- `pointCloudShapes` contains sampled shape generators used by the current point-cloud backend.
