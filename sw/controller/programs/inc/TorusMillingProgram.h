#ifndef TRANSMUTER_CONTROLLER_PROGRAMS_TORUS_MILLING_H
#define TRANSMUTER_CONTROLLER_PROGRAMS_TORUS_MILLING_H

#include "MillingProgramContext.h"
#include "PathAlgorithmConfig.h"
#include "programIf.h"

namespace transmuter::controller::programs
{

/** One raster operation from dodecahedron stock toward a torus target. */
class TorusMillingProgram final : public ProgramIf
{
   public:
    void execute() override;

   private:
    MillingProgramContext createMillingContext() const;
    pathgenerator::PathAlgorithmConfig pathConfig() const;
};

}  // namespace transmuter::controller::programs
#endif
