#include "controller.h"

#include "FiveAxisNormalMillingProgram.h"
#include "InteractiveTorusMillingProgram.h"
#include "TorusMillingProgram.h"
#include "programManager.h"
namespace transmuter::controller
{
void runProgram(const std::string& name)
{
    ProgramManager manager;
    manager.add<programs::TorusMillingProgram>("torusMilling");
    manager.add<programs::InteractiveTorusMillingProgram>("interactiveTorusMilling");
    manager.add<programs::FiveAxisNormalMillingProgram>("fiveAxisNormalMilling");
    manager.run(name);
}
}  // namespace transmuter::controller
