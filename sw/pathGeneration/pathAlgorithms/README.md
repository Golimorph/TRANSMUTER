# Path algorithms

Every algorithm implements `PathAlgorithmIf`. The interface retains immutable stock, target model,
and tool inputs, and makes the generated outputs explicit through `generate` and
`PathGenerationResult`.

Algorithms ask `GeometryIf` for geometric properties and subtract posed tool geometry directly
from their mutable workpiece snapshot. They publish ordered poses through `ToolPathIf` and do not
inspect point-cloud storage.

Each algorithm has its own folder containing its public `inc` and private implementation `src`
files. `ThreeAxisSpiralAlgorithm` coordinates the configured layers and circles.
`ThreeAxisSpiralCuttingOperation` receives the stock, target model, tool, and live tool path, then
owns the generation state and cutting mechanics for one run. Their interface makes the
configuration mapping explicit: pass spacing divides a layer into concentric circles, point
spacing samples each circle, and layer spacing selects the next cutting depth.

The spiral algorithm's top-level workflow cuts counter-clockwise concentric circles from the stock
edge to its center, retracts and moves safely to the next depth, and repeats. Its private
three-axis cutting operation owns the vertical lifting rule and raises the tool until the model's
geometry-level intersection query reports a clear pose. A stop completes the current layer before
generation exits.
