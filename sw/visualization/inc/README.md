# Visualization interfaces

`Visualizer` owns playback timing and material-removal progression. `Renderer` owns window and
drawing concerns. Both receive solids through `GeometryIf`; only ordered tool-path storage uses the
concrete `PointCloud` type.
