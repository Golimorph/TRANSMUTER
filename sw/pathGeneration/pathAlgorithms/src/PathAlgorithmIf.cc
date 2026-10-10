#include "PathAlgorithmIf.h"

#include "trace.h"

namespace transmuter::pathgenerator::algorithms
{
PathAlgorithmIf::PathAlgorithmIf(const geometry::MillingScene& millingScene, MachineKinematicsIf& machine)
    : m_scene(millingScene), m_machineKinematics(machine)
{
    m_workpiece = m_scene.workpiece()->clone();
    INFO("Path generation algorithm ready.");
}
const geometry::MillingScene& PathAlgorithmIf::scene() const
{
    return m_scene;
}
const geometry::GeometryIf& PathAlgorithmIf::operationStock() const
{
    return *m_scene.workpiece();
}
const geometry::GeometryIf& PathAlgorithmIf::model() const
{
    return *m_scene.model();
}
const geometry::GeometryIf& PathAlgorithmIf::toolModel() const
{
    return *m_scene.toolModel();
}
geometry::GeometryIf& PathAlgorithmIf::workpiece()
{
    return *m_workpiece;
}
std::shared_ptr<const geometry::GeometryIf> PathAlgorithmIf::finishedWorkpiece() const
{
    return m_workpiece;
}
MachineKinematicsIf& PathAlgorithmIf::machine()
{
    return m_machineKinematics;
}
}  // namespace transmuter::pathgenerator::algorithms
