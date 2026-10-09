#include "controller.h"
#include "trace.h"

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        ERROR(
            "Usage: ./transmuter <program_name>\nPrograms: torusMilling, quickTorusMilling, "
            "fiveAxisNormalMilling");
    }
    transmuter::controller::runProgram(argv[1]);
    return 0;
}
