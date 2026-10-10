#pragma once

#include "PathGenerationContext.h"
#include "millingScene.h"

namespace transmuter::controller::programs
{
struct MillingProgramContext
{
    geometry::MillingScene geometryContext;
    pathgenerator::PathGenerationContext pathContext;
};
}  // namespace transmuter::controller::programs
