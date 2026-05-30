#include <string>

#include "boxMillingProgram.h"
#include "programManager.h"
#include "trace.h"

struct Arguments
{
    std::string programName;
};

static Arguments parseArguments(int argc, char* argv[])
{
    if (argc < 2)
    {
        ERROR(
            "Usage: ./transmuter <program_name>\n"
            "Example: ./transmuter boxMilling\n"
        );
    }
    Arguments args;
    args.programName = argv[1];
    return args;
}

transmuter::controller::ProgramManager createProgramManager()
{
    transmuter::controller::ProgramManager programManager;
    programManager.add<transmuter::controller::programs::BoxMillingProgram>("boxMilling");
    return programManager;
}

int main(int argc, char* argv[])
{
    Arguments args = parseArguments(argc, argv);
    transmuter::controller::ProgramManager programManager = createProgramManager();
    INFO("Launcher: Attempting to run program '" + args.programName + "'...");
    programManager.run(args.programName);
    return 0;
}