# Programs

Programs independently choose geometry, machine kinematics, algorithm, and path settings through
public interfaces. The registered examples are `torusMilling`, `interactiveTorusMilling`, and
`fiveAxisNormalMilling`; `make run` selects the interactive example. The former `boxMilling` program is
named `torusMilling` because it mills a torus from a dodecahedron.

`InteractiveTorusMillingProgram::execute()` shows the interactive flow: wait for a user milling
request, cut the intersection of the selected volume and current workpiece, then play back the
combined path. The selection UI supplies the volume, algorithm, and path settings; the program
combines that volume with the scene geometry and passes the resulting stock to the path generator.
Each completed operation is appended to the program context's machine path.

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
