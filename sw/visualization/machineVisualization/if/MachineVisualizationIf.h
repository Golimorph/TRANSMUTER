#pragma once
#include <memory>

#include "MachineKinematicsIf.h"
#include "machineDrawing.h"
namespace transmuter::visualization
{
class MachineVisualizationIf
{
   public:
    virtual MachineDrawing getMachineDrawing(const geometry::MachineAxes& axes) const = 0;
    virtual ~MachineVisualizationIf() = default;
};
std::unique_ptr<MachineVisualizationIf> createMachineVisualization(std::shared_ptr<const pathgenerator::MachineKinematicsIf> machineKinematics, MachineDrawingConfig config = {});
}  // namespace transmuter::visualization
