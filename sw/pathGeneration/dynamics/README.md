# Machine dynamics

`if/MachineKinematicsIf.h` is the public machine boundary. `machineFactory.h` constructs a fixed
three-axis or trunnion five-axis implementation from the program's `MachineConfig` and optional
`MachineMotionConfig`. The latter sets interpolation sampling and requested-path tolerance once
when the machine is created; algorithms do not pass these values with each proposed move.
`MachineKinematicsIf::propose` checks one requested tool pose against the machine's accepted path.
On acceptance, it records the axis endpoint before returning. Rejection leaves the path unchanged.
The machine also maps saved axes into generic display poses. `if/MachinePathIf.h` is an
ordered, representation-independent view of physical moves. Only accepted endpoints are recorded.

The private `MachineMechanicsIf` supplies forward kinematics and initial inverse solutions.
`MachineAxisOptimizer` searches subsets of movable axes with a deterministic numerical fit. It
prefers fewer moving axes, then a larger sum of squared accumulated per-axis tool-motion
contributions. Rotary motion is converted to millimetres using configured machine dimensions.
It bounds detours and validates joint interpolation against the requested workpiece curve.
The machine reconstructs preload from its accepted axis history, then tries ranked axis solutions
for the proposed segment. Geometry contact and material removal remain with the algorithm.
The search is bounded and cannot prove a global optimum over an entire finished program.
Interpolation samples remain private; only accepted machine endpoints are published.

The example machine's trunnion uses `Ry(A) * Rz(B)`, a joint 100 mm above the bed, and a 50 mm
workpiece-center offset. A=-90 degrees points local +Z toward machine -X. The program explicitly
chooses those mounting offsets and the X/Y backlash mask in `MachineConfig`. The travel-direction
force implementation lives behind `toolForce/if/ToolForceIf.h` and its own factory; the machine
factory injects that interface into the generic planning wrapper. The
machine drawing has its own presentation geometry in `visualization/machineVisualization`.

The force estimate returns the workpiece reaction to equal opposing-travel and sideways cutter
loads for fixed right-hand spindle rotation. It does not use contact geometry, chip load, or machine dynamics. The checks are a
diagnostic model rather than machine-ready proof.
