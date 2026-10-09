# Programs

Programs independently choose geometry, machine kinematics, algorithm, and path settings through
public interfaces. The registered examples are `torusMilling`, `quickTorusMilling`, and
`fiveAxisNormalMilling`; `make run` selects the quick example. The former `boxMilling` program is
named `torusMilling` because it mills a torus from a dodecahedron.

`QuickTorusMillingProgram::execute()` shows the stage sequence: start roughing, pass its resulting
workpiece as fine cutting's local stock, then play back the combined path. The program keeps one `PathGenerator`
slot and replaces it between stages. Each operation has its own live path, shown as it grows. When
an operation finishes, the program appends it to the combined path. The machine checks the
clearance transfer against the combined axis history before those transfer points are recorded. The transfer is
checked against the stock remaining after roughing; no complete replay runs after generation.

| Stage | Point spacing | Pass spacing | Layer spacing |
| --- | --- | --- | --- |
| Rough | 10 mm | 20 mm | 40 mm |
| Fine | 5 mm | 10 mm | 10 mm |

Both stages protect the same torus through `GeometryIf`. Fine cutting uses roughing's output as its
stock. These are coarse preview settings, not a measured speedup or a guarantee of full torus
coverage. Three-axis access cannot reach every torus undercut. The transfer's local check also
cannot prove that the independently planned stages have identical preload history at their join.

`TorusMillingProgram` runs one raster operation. `FiveAxisNormalMillingProgram` runs the experimental
surface-peeling algorithm on the trunnion. Its 20 mm cutting region is enclosed by an 80 mm tool
assembly so the shank is checked for collision without removing stock. That planner can exhaust its
candidate orientations before achieving full coverage; its result note says how many cuts were
accepted and rejected.

Every generated move is submitted to the machine before it is recorded. Raster and spiral call
`ERROR` if a proposed move is rejected. Five-axis candidate rejection triggers another geometric
candidate. The program's one stop signal is shared with the active generator and machine motion
check. The scene workpiece used by the viewer is cut during playback after both operations finish;
each algorithm changes only its private workpiece while planning.

Generated machine paths retain X/Y/Z/A/B positions; workpiece poses are derived from those axes.
Each completed result contains an independent `MachinePathIf` snapshot. Press **W** for workpiece coordinates
or **M** for the animated machine view. The program loop processes window events and renders frames
on the main thread.
