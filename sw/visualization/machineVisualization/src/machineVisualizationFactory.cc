#include <utility>

#include "BridgeportMachineVisualization.h"
#include "MachineVisualizationIf.h"

namespace transmuter::visualization
{
std::unique_ptr<MachineVisualizationIf> createMachineVisualization(std::shared_ptr<const pathgenerator::MachineKinematicsIf> machineKinematics, MachineDrawingConfig config)
{
    return std::make_unique<BridgeportMachineVisualization>(std::move(machineKinematics), config);
}
}  // namespace transmuter::visualization
