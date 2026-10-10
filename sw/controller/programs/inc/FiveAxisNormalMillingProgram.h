#pragma once
#include "MillingProgramContext.h"
#include "PathAlgorithmConfig.h"
#include "programIf.h"

namespace transmuter::controller::programs
{
class FiveAxisNormalMillingProgram final : public ProgramIf
{
   public:
    void execute() override;

   private:
    MillingProgramContext createMillingContext() const;
    pathgenerator::PathAlgorithmConfig pathConfig() const;
};
}  // namespace transmuter::controller::programs
