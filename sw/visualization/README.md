# Visualization

`if/Visualizer.h` exposes scene presentation and playback controls; `VisualizerConfig.h` contains the
settings. `Visualizer` coordinates the displayed frame and delegates playback to the private
`playback/MachinePathPlayback` component. That component owns timing, pose interpolation and displayed
material removal, and contains no window or OpenGL calls. `inc/Renderer.h` and `src/Renderer.cc` own
window input, camera control and OpenGL drawing. Solids are consumed through `GeometryIf`, paths through `MachinePathIf`, kinematics through
`MachineKinematicsIf`, and the machine sketch through `MachineVisualizationIf`.

Programs call `processEvents()` from their main loop. It processes window input and renders one frame
on the same thread that created the window and owns the OpenGL context. Playback and camera changes
therefore follow the program's normal event flow without a separate render worker.

Press **W** to select workpiece coordinates or **M** to select machine coordinates. The window title
shows the active frame and the controls. Horizontal mouse drag rotates about scene Z (azimuth/phi); vertical drag changes the polar
angle (theta), with downward drag decreasing theta and upward drag increasing it. Theta is clamped near the poles to avoid flipping, and the wheel zooms. The initial
frame is configurable through `VisualizerConfig::initialCoordinates`.

In workpiece view the stock stays fixed and the tool moves around it. In machine view the X/Y table,
A/B trunnion, workpiece and spindle Z move according to the stored machine trajectory. The path overlay
remains attached to the workpiece, so it is a reference to the cuts on the part rather than a trace
of the spindle in room coordinates. While generation runs, each frame reads the active operation's
preview path and displays its latest point and resolved machine pose. Completed operations join the
scene-owned path, which is drawn separately from the preview. Switching views does not modify the path or stock state.

Playback interpolates recorded machine axes and obtains both scene frames and the tool's
workpiece-relative pose through forward kinematics. Cutting moves remove material through
`GeometryIf::cut`; rapid moves only reposition the tool. Every displayed tool pose is derived
from recorded axes.

`machineVisualization` supplies a stylized Bridgeport-style machine as cached triangle meshes with
instance transforms and colors. It contains no OpenGL or window code. The OpenGL renderer draws those
parts with indexed triangle calls and never reconstructs machine kinematics.
