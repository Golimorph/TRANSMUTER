# Machine visualization

`MachineVisualizationIf::getMachineDrawing(MachineAxes)` returns a `MachineDrawing`: shared immutable
triangle meshes, transforms and colors for each part. Vertices contain positions/normals and triangle
indices use 32-bit integers. The format can be consumed by OpenGL, another renderer, or a mesh exporter.
It does not contain OpenGL resources, window handles, callbacks, or renderer-specific commands.

The factory receives the public kinematics interface and small drawing settings (workpiece base plane
and tool length). `BridgeportMachineVisualization` describes the base, column, ram, knee, Y saddle,
X table with slots, A bearings/cradle, B platter and Z spindle. `MachineMeshBuilder` creates reusable
unit box/cylinder meshes once. Only transforms and instance data change per frame.

All implementation classes are private in matching files. Geometry and articulation use kinematic
frames from `MachineKinematicsIf`; the drawing does not duplicate inverse-kinematic calculations.
The frame dimensions are illustrative. This sketch is not the machine/fixture collision model used
to certify machining clearance.
