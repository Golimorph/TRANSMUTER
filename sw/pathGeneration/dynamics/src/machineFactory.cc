#include "machineFactory.h"

#include "FixedThreeAxis.h"
#include "MachineKinematics.h"
#include "MachinePath.h"
#include "TrunnionFiveAxis.h"
#include "toolForceFactory.h"
namespace transmuter::pathgenerator
{
std::shared_ptr<MachineKinematicsIf> createFixedThreeAxisMachine(const MachineConfig& config, MachineMotionConfig motionConfig)
{
    return std::make_shared<MachineKinematics>(std::make_shared<FixedThreeAxis>(config), travelDirectionToolForce(), config, motionConfig);
}
std::shared_ptr<MachineKinematicsIf> createTrunnionFiveAxisMachine(const MachineConfig& config, MachineMotionConfig motionConfig)
{
    return std::make_shared<MachineKinematics>(std::make_shared<TrunnionFiveAxis>(config), travelDirectionToolForce(), config, motionConfig);
}
std::shared_ptr<MachinePathIf> createMachinePath()
{
    return std::make_shared<MachinePath>();
}
}  // namespace transmuter::pathgenerator
