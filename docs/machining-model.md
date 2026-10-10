# Backlash-aware machining model

The implementation supports studying preload-constrained tool paths. It does not yet establish that
an arbitrary stock can be completely machined into an arbitrary target, nor that a path is ready for
the physical machine. Geometry and the machine are separate interfaces. The machine implementation owns kinematics,
cutting-force estimation, and backlash validation.

## Force and feed conventions

A tool pose contains its origin and the axis pointing from tip toward shank, expressed in workpiece
coordinates. The force model assumes right-hand rotation about that axis.

When the cutter engages stock, let `t` be the normalized travel between consecutive tool poses and
`a` the normalized tool axis at the new pose. The default relative force **on the workpiece** is
`(t - normalize(t cross a)) / sqrt(2)`. For tool travel along +X and an axis along +Z, this gives
`(1, 1, 0) / sqrt(2)`. The corresponding force on the cutter is `(-1, -1, 0) / sqrt(2)`.
The distinction matters because the physical X/Y table moves opposite to workpiece-relative tool
travel. A purely axial move has no defined sideways direction, so this model uses
`t` alone. A zero-length move in material is inconclusive. Contact is checked separately; the
force model does not inspect contact geometry.

For a backlash axis, the check is `forceComponent * lastEstablishedFeedSign <= forceTolerance`.
The check is performed separately for X and Y; a negative dot product between whole feed and force
vectors is insufficient. For example, feed `(1,1)` and force `(-2,1)` oppose overall but still load Y
in the same direction as travel. Zero force on an axis is allowed. After an X move, stationary X
retains that preload direction during subsequent Y motion. Reversal clears established preload until
the configured take-up distance has been traversed. Movement in air is included.

The private default cutting-force model is a qualitative travel-direction model, not a calibrated
tooth/edge contact integral. It ignores engagement shape, helix, flute phase, chip thickness,
friction, dynamics, and force variation between teeth. Its opposing-feed component includes axial
travel when present; it does not predict actual axial cutting loads. Replacing the model and refining
geometry are necessary before drawing stronger physical conclusions. Experimental mechanistic force
models distinguish tangential, normal, and axial contributions; see
[Milling Force Modeling: A Comparison of Two Approaches](https://mtrc.utk.edu/wp-content/uploads/sites/45/2019/09/force_model.pdf).

Conventional/up milling can oppose the feed, whereas down milling can pull a drive with backlash
into the cut. Engagement still determines the component forces: neither counter-clockwise circles
nor unidirectional raster rows alone prove X/Y preload, particularly at entries, links, and changing
stock boundaries. [Sandvik's slot-milling guidance](https://www.sandvik.coromant.com/en-gb/knowledge/milling/groove-or-slot-milling)
explains why down-milling feed forces make rigidity and elimination of backlash important.

## Kinematics

The fixed three-axis model requires +Z tool orientation and holds A=B=0; its physical table
X/Y positions have the opposite sign from workpiece-relative tool X/Y.
The five-axis example selects the configured intersecting-axis table with
`workpieceRotation = Ry(A) * Rz(B)`. Its A/B joint is `(0, 0, 100)` mm in machine coordinates,
100 mm above the bed. The workpiece-frame origin is offset by `(0, 0, 50)` mm in the local B frame.
Before XY table compensation, the requested tool origin is
`jointPositionInMachine + workpieceRotation * (workpieceCenterFromJoint + toolOrigin)`.

A = 0 is the three-axis orientation. A = -90 degrees maps the stock's local +Z axis onto machine -X,
so B then rotates about machine -X. With the default axial mounting and X=Y=0, the workpiece center is
`(0, 0, 150)` at A=0 and `(-50, 0, 100)` at A=-90 degrees. A moves the center; B alone does not move
an on-axis center. Nonzero X/Y components of `workpieceCenterFromJoint` represent eccentric mounting
and make B move the center as well.

The private trunnion model supplies initial inverse branches and default mathematical A limits
within `[-pi, pi]`; they are not measured machine travel. The machine optimizer then searches
combinations of movable axes using a deterministic numerical fit. It prefers fewer moving axes,
then concentrates tool-motion contribution among feasible local solutions. Its detour bound prevents
arbitrarily large, cancelling axis moves from winning the squared-contribution metric. The search
keeps alternative endpoints for each requested move. This is a local curve search,
not a global optimizer over the whole program.

The stored axis convention is physical table X/Y travel and spindle-tip Z, in millimetres, with A/B
in continuous radians. For a requested workpiece point `p`, let
`u = jointPositionInMachine + R * (workpieceCenterFromJoint + p)`.
The inverse solution is `X=-u.x`, `Y=-u.y`, `Z=u.z`. The spindle stays on machine X=Y=0.
Forward kinematics places the workpiece with
`T(X,Y,0) * T(joint) * Ry(A) * Rz(B) * T(mountingOffset)` and the tool tip with `T(0,0,Z)`.
The workpiece load maps directly into physical table X/Y loads; the spindle Z load is its reaction.

Every generated path point preserves its selected X/Y/Z/A/B solution, and `MachinePathIf` records
one physical endpoint for each tool-path endpoint. Validation samples between those endpoints
without adding published path points. Playback interpolates the recorded axes, including full
continuous rotary turns, and uses forward kinematics to recover the workpiece-relative tool pose.
The sampled machine curve is checked against the requested workpiece curve as well as its endpoints.
This makes the axis trajectory explicit, although controller-specific interpolation, rates,
accelerations, calibrated limits, fixtures, and machine-body collision checking remain outside the
example. The inverse-kinematics literature also distinguishes machine architectures:
[Farouki, Han and Li](https://escholarship.org/uc/item/7d5066mb).

## Surfaces and access

A dodecahedron and a torus have different topology, so an everywhere regular one-to-one surface
mapping cannot describe the creation of the hole. The experimental planner instead repeatedly samples
the boundary of the remaining stock and removes admissible shallow cuts. It favours exposed normals,
then tries alternate axes for regions such as the torus hole. These are approximate normals on sampled
stock, not smooth CAD-surface normals. It has a finite orientation search and may stop with substantial
material remaining; unresolved attempts do not prove that machining is impossible.

The tool assembly is checked against the target along the approach, cut, retract, and links. Its
non-cutting part is also checked against remaining stock. A long cutting cylinder alone would not
represent a shank collision correctly, since it would wrongly allow the entire shank to remove stock.
The five-axis example therefore separates the 20 mm cutting length from the 80 mm assembly.
Tool-orientation accessibility is an established part of five-axis planning; see
[Collision-free Tool Path Generation Using 2-Dimensional C-Space](https://www.ims.mce.uec.ac.jp/papers/cam/5axis/1997-06e/index.html).

## Reading results

During generation, each algorithm owns a workpiece cloned from its operation's input stock. It
checks stock and target geometry, proposes a point, and asks the machine to resolve and validate
the axis motion. Only an accepted axis endpoint is appended to the machine path; then the
algorithm removes cutter material from its private workpiece. A rejected proposal leaves
that path and workpiece unchanged. Raster and spiral call `ERROR` on rejection; the
experimental five-axis planner tries another candidate. The quick program passes roughing's output
as fine cutting's local stock and checks transfers against the combined axis history and the
remaining material before joining operations. The viewer's scene workpiece changes during playback.

The machine samples joint interpolation and checks the requested curve and axis preload. It
assumes a cutting move is loaded and a rapid move is unloaded. Raster and spiral assign these
types from their planned operations; the experimental five-axis planner checks contact.
Sampling cannot certify every point of a physical continuous path.
Finishing spacing finer than the stock/target sampling cannot recover missing geometric details.
The example torus uses 3 mm geometry samples.

## Module boundaries

The example programs independently own dimensions, tools, machine choice, and stage settings.
Each program composes a geometry `MillingScene` with a path-generation context that holds the
selected machine and combined path, shared by generation and visualization. The program chooses
mounting offsets and backlash axes through `MachineConfig`; the
machine's `MachineMotionConfig` controls sampling and curve tolerance.
`PathAlgorithmConfig` controls requested geometry. Geometry sweeps use the algorithm's point
spacing; machine-motion checks use the machine's sampling spacing.
Algorithms receive an input scene and construct geometry through `GeometryIf`. Their cutting
operations send target-adjusted workpiece points to `MachineKinematicsIf::propose`.
The machine ranks axis motions and checks each sampled segment without seeing any geometry;
the machine records accepted axis endpoints, then the algorithm cuts its workpiece.
The algorithm returns the completed machine path in `PathGenerationResult`. `PathGenerator` owns asynchronous
execution. The viewer shows each operation's path during generation; the program joins that path
to the combined scene path when the operation finishes.

Raster follows the original layers-and-rows structure. Its cutting operation lifts requested poses that intersect the
target using `GeometryIf` directly. At the low-level
move boundary, the machine checks motion and reports unresolved constraints; it does not prescribe
row geometry or machine joints.

The five-axis operation traverses exposed surfaces and proposes geometric candidate curves. Its
`geometry` folder supplies surface sampling and access geometry without stock mutation or inverse
kinematics. A forked machine planner and workpiece clone check each candidate point in order;
the algorithm commits the whole candidate only when all points are accepted. The tool-force
interface and implementation have their own `toolForce` folder;
only the machine constructs and consults the force model.

Geometry owns cloning, intersections, cutting and material sampling. No parallel stock-simulation
backend is needed. Roughing and finishing both check the actual target through `GeometryIf`;
their spacing determines how coarsely each stage covers the stock.

The visualizer delegates simulation playback to `playback/MachinePathPlayback` and drawing to `Renderer`.
Machine drawing remains isolated under `machineVisualization`, producing cached meshes and transforms
through `MachineVisualizationIf`. Rendering, playback and motion checks use the same saved axes and
machine interface.
