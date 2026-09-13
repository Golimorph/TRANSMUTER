# Programs

Programs coordinate geometry creation, path generation, and visualization without implementing
those concerns themselves. Scene solids are stored through `GeometryIf`, while a shared live tool
path connects generation to visualization.

`BoxMillingProgram` is the current minimal workflow: create a solid cylindrical stock and box
target, generate one 3-axis spiral path, then replay that path while material disappears from the
visible workpiece.

The `if` directory contains the program interface; `inc` and `src` contain one implementation per
workflow.
