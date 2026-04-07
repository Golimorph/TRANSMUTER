#ifndef TRANSMUTER_CONTROLLER_PROGRAMS_BOX_MILLING_H
#define TRANSMUTER_CONTROLLER_PROGRAMS_BOX_MILLING_H

#include <memory>

#include "millingScene.h"
#include "programIf.h"

namespace transmuter::controller::programs
{

class BoxMillingProgram : public ProgramIf
{
   public:
    BoxMillingProgram() = default;
    ~BoxMillingProgram() override = default;

    void execute() override;

   private:
    transmuter::geometry::MillingScene createMillingScene() const;
};

}  // namespace transmuter::controller::programs
#endif