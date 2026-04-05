#include "boxMilling.h"
#include "controller.h"

int main()
{
    // 1. Setup the Controller
    transmuter::controller::Controller bernardo;

    // 2. Register the programs
    bernardo.add<transmuter::controller::programs::BoxMilling>("box");

    // 3. Run the selected logic
    // This calls boxMilling->execute(), which takes over the process
    bernardo.run("box");
    return 0;
}