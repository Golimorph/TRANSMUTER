#include "controller.h"

#include "FiveAxisNormalMillingProgram.h"
#include "QuickTorusMillingProgram.h"
#include "TorusMillingProgram.h"
#include "programManager.h"
namespace transmuter::controller
{
void runProgram(const std::string& name)
{
    ProgramManager manager;
    manager.add<programs::TorusMillingProgram>("torusMilling");
    manager.add<programs::QuickTorusMillingProgram>("quickTorusMilling");
    manager.add<programs::FiveAxisNormalMillingProgram>("fiveAxisNormalMilling");
    manager.run(name);
}
}  // namespace transmuter::controller
