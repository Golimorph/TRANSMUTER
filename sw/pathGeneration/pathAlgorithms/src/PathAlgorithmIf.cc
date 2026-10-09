#include "PathAlgorithmIf.h"

#include "trace.h"

namespace transmuter::pathgenerator::algorithms
{
PathAlgorithmIf::PathAlgorithmIf(const geometry::MillingScene& millingScene) : m_scene(millingScene)
{
    if (!m_scene.stock || !m_scene.model || !m_scene.toolModel)
        ERROR("Path generation requires stock, target and tool geometry.");
    m_workpiece = m_scene.stock->clone();
    INFO("Path generation algorithm ready.");
}
const geometry::MillingScene& PathAlgorithmIf::scene() const
{
    return m_scene;
}
const geometry::GeometryIf& PathAlgorithmIf::stock() const
{
    return *m_scene.stock;
}
const geometry::GeometryIf& PathAlgorithmIf::model() const
{
    return *m_scene.model;
}
const geometry::GeometryIf& PathAlgorithmIf::toolModel() const
{
    return *m_scene.toolModel;
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
    return *m_scene.machineKinematics;
}
}  // namespace transmuter::pathgenerator::algorithms
