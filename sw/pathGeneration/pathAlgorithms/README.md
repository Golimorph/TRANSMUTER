# Path algorithms

Every algorithm implements `PathAlgorithmIf`. The interface retains immutable stock, target model,
and tool inputs, and makes the generated outputs explicit through `generate` and
`PathGenerationResult`.

Algorithms ask `GeometryIf` for geometric properties and subtract posed tool geometry directly
from their mutable workpiece snapshot. They publish ordered poses through `ToolPathIf` and do not
inspect point-cloud storage.

Each algorithm has its own folder containing its public `inc` and private implementation `src`
files. `ThreeAxisRasterAlgorithm` coordinates depth layers and unidirectional rows. Every cutting
row moves from low X to high X; the tool retracts and returns at safe height before beginning the
next row. `ThreeAxisRasterCuttingOperation` owns row transitions, model avoidance, and workpiece
cutting for one run. Unidirectional motion in workpiece coordinates is not by itself a backlash
guarantee; a future machine-aware planner must validate the resulting linear-axis motion after
applying the machine kinematics.

`ThreeAxisSpiralAlgorithm` coordinates the configured layers and circles.
`ThreeAxisSpiralCuttingOperation` receives the stock, target model, tool, and caller-owned path
output, then owns the generation state and cutting mechanics for one run. Their interface makes the
configuration mapping explicit: pass spacing divides a layer into concentric circles, point
spacing samples each circle, and layer spacing selects the next cutting depth.

The spiral algorithm's top-level workflow cuts counter-clockwise concentric circles from the stock
edge to its center, retracts and moves safely to the next depth, and repeats. Its private
three-axis cutting operation owns the vertical lifting rule and raises the tool until the model's
geometry-level intersection query reports a clear pose. A stop completes the current layer before
generation exits.
