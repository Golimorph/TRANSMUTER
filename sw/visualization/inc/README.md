# Visualization interfaces

`Visualizer` owns playback timing and material-removal progression. `Renderer` owns window and
drawing concerns. Both receive solids through `GeometryIf` and ordered tool poses through
`ToolPathIf`.
