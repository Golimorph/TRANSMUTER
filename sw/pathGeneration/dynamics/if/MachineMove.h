#pragma once

#include "machineAxes.h"

namespace transmuter::pathgenerator
{
enum class MotionType
{
    CUTTING,
    RAPID
};

struct MachineMove
{
    geometry::MachineAxes axes;
    MotionType motionType = MotionType::CUTTING;
};
}  // namespace transmuter::pathgenerator
