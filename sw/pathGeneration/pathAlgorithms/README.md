# Path algorithms

Every algorithm implements `PathAlgorithmIf`. The interface retains immutable stock, target model,
and tool inputs, and makes the generated outputs explicit through `generate` and
`PathGenerationResult`.

Algorithms ask `GeometryIf` for geometric properties and use geometry-domain operation interfaces
for collision and material removal. They do not inspect point-cloud storage. The one intentional
`PointCloud` dependency is the live tool path, which is a sequence of tool poses rather than a
solid representation.

Each algorithm has its own folder containing its public `inc` and private implementation `src`
files. `ThreeAxisSpiralAlgorithm` interprets generic scene geometry as an algorithm-specific
cylindrical milling volume and coordinates its layers and circles.
`ThreeAxisSpiralCuttingOperation` receives that explicit volume plus a local workpiece and owns the
tool-motion, collision-avoidance, and material-removal mechanics for one generation run. Their
interface makes the configuration mapping explicit: pass spacing divides a layer into concentric
circles, point spacing samples each circle, and layer spacing selects the next cutting depth.

The spiral algorithm's top-level workflow cuts counter-clockwise concentric circles from the stock
edge to its center, retracts and moves safely to the next depth, and repeats. At every sampled pose,
the cutting operation asks the geometry layer to lift the vertical tool just enough to avoid the
target model. A stop completes the current layer before generation exits.
