#pragma once
#include "machineDrawing.h"
namespace transmuter::visualization
{
class MachineMeshBuilder
{
   private:
    friend class BridgeportMachineVisualization;

    static std::shared_ptr<const MachineMesh> box();
    static std::shared_ptr<const MachineMesh> cylinder();

    static void addTriangle(MachineMesh& mesh, const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c);
};
}  // namespace transmuter::visualization
