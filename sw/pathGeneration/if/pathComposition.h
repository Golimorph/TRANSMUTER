#pragma once

#include <atomic>

#include "MachinePathIf.h"
#include "millingScene.h"

namespace transmuter::pathgenerator
{
/** Join completed +Z operations through machine-checked clearance moves. */
void appendThreeAxisOperationPath(const MachinePathIf& operation, MachinePathIf& combined, const geometry::MillingScene& transitionScene, double geometrySampleSpacing, const std::atomic<bool>& stop);
}  // namespace transmuter::pathgenerator
