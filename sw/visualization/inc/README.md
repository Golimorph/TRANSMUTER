# Private visualization implementation

`Renderer` owns OpenGL drawing, window input, and synchronized camera settings. The main thread
creates the window and processes events; the visualization worker owns its OpenGL context while
rendering frames. Its headers are private to this
module. Consumers use `../if/Visualizer.h`; renderer-independent machine drawing is exposed by
`../machineVisualization/if/MachineVisualizationIf.h`.
