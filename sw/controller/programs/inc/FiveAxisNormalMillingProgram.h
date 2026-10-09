#pragma once
#include "PathAlgorithmConfig.h"
#include "millingScene.h"
#include "programIf.h"

namespace transmuter::controller::programs
{
class FiveAxisNormalMillingProgram final : public ProgramIf
{
   public:
    void execute() override;

   private:
    geometry::MillingScene createMillingScene() const;
    pathgenerator::PathAlgorithmConfig pathConfig() const;
};
}  // namespace transmuter::controller::programs
