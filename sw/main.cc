#include "boxMillingProgram.h"
#include "programManager.h"
#include "trace.h"

transmuter::controller::ProgramManager createProgramManager()
{
    transmuter::controller::ProgramManager programManager;
    programManager.add<transmuter::controller::programs::BoxMillingProgram>("boxMilling");
    return programManager;
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        ERROR(
            "Usage: ./transmuter <program_name>\nExample: ./transmuter box , where 'box' is the "
            "name of the program to run.");
    }
    std::string requestedProgram = argv[1];

    transmuter::controller::ProgramManager programManager = createProgramManager();

    INFO("Launcher: Attempting to run program '" + requestedProgram + "'...");
    programManager.run(requestedProgram);

    return 0;
}