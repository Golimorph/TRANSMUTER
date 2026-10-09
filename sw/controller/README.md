# Controller

The launcher calls the public `if/controller.h` entry point. Program registration and concrete
workflow selection belong to `src/controller.cc`; program implementations and `ProgramManager`
remain private. Workflows use public geometry, path-generation, and visualization contracts.
