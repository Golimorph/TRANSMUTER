#pragma once

#include "MachineConfig.h"
#include "MachineKinematicsIf.h"
#include "MachineMotionConfig.h"

namespace transmuter::pathgenerator
{
std::shared_ptr<MachineKinematicsIf> createFixedThreeAxisMachine(const MachineConfig& config, MachineMotionConfig motionConfig = {});
std::shared_ptr<MachineKinematicsIf> createTrunnionFiveAxisMachine(const MachineConfig& config, MachineMotionConfig motionConfig = {});
std::shared_ptr<MachinePathIf> createMachinePath();
}  // namespace transmuter::pathgenerator
