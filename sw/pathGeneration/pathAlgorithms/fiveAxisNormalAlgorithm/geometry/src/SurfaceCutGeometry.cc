#include "SurfaceCutGeometry.h"

#include <algorithm>
#include <cmath>

#include "toolPose.h"
#include "trace.h"

namespace transmuter::pathgenerator::algorithms
{
SurfaceCutGeometry::SurfaceCutGeometry(const geometry::MillingScene& scene, const PathAlgorithmConfig& config)
    : m_scene(scene), m_config(config)
{
    configureClearance();
}
geometry::Vertex SurfaceCutGeometry::initialPose() const
{
    return {m_center + Eigen::Vector3d::UnitZ() * m_clearanceRadius, Eigen::Vector3d::UnitZ()};
}
void SurfaceCutGeometry::configureClearance()
{
    const auto stockBox = m_scene.stock->bounds(), modelBox = m_scene.model->bounds(),
               toolBox = assembly().bounds();
    if (!stockBox.isValid || !modelBox.isValid || !toolBox.isValid)
    {
        ERROR("Normal-surface generation requires nonempty geometry.");
    }
    const Eigen::Vector3d minimum = stockBox.min.cwiseMin(modelBox.min),
                          maximum = stockBox.max.cwiseMax(modelBox.max);
    m_center = (minimum + maximum) * 0.5;
    const double toolReach = toolBox.min.cwiseAbs().cwiseMax(toolBox.max.cwiseAbs()).norm();
    m_clearanceRadius = (maximum - minimum).norm() * 0.5 + toolReach + m_config.surfaceSampleSpacing;
}
const geometry::GeometryIf& SurfaceCutGeometry::assembly() const
{
    return m_scene.toolAssemblyModel ? *m_scene.toolAssemblyModel : *m_scene.toolModel;
}
geometry::Vertex SurfaceCutGeometry::clearancePose(const geometry::Vertex& pose) const
{
    const Eigen::Vector3d offset = pose.pos - m_center;
    const double along = offset.dot(pose.normal);
    const double discriminant = along * along + m_clearanceRadius * m_clearanceRadius - offset.squaredNorm();
    const double distance = -along + std::sqrt(std::max(0.0, discriminant));
    return {pose.pos + distance * pose.normal, pose.normal};
}
std::optional<std::vector<SurfaceCutGeometry::Move>> SurfaceCutGeometry::candidate(const geometry::Vertex& current, const geometry::Vertex& surface, const Eigen::Vector3d& axis, const Eigen::Vector3d& tangent) const
{
    const double length = m_config.passSpacing;
    const Eigen::Vector3d center = surface.pos -
                                   axis * (m_config.layerSpacing + m_scene.toolModel->bounds().min.z());
    const geometry::Vertex start{center - tangent * length * 0.5, axis};
    const geometry::Vertex finish{center + tangent * length * 0.5, axis};
    std::vector<Move> path;
    appendClearanceLink(path, current, clearancePose(start));
    path.push_back({start, MotionType::CUTTING});
    path.push_back({finish, MotionType::CUTTING});
    path.push_back({clearancePose(finish), MotionType::CUTTING});
    if (!cuttingEndpointsClear(path))
    {
        return std::nullopt;
    }
    return path;
}
void SurfaceCutGeometry::appendClearanceLink(std::vector<Move>& path, const geometry::Vertex& current, const geometry::Vertex& destination) const
{
    const Eigen::Vector3d from = (current.pos - m_center).normalized();
    const Eigen::Vector3d to = (destination.pos - m_center).normalized();
    const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(from, to);
    const size_t count = std::max<size_t>(1, static_cast<size_t>(std::ceil(rotation.angularDistance(Eigen::Quaterniond::Identity()) / 0.05)));
    for (size_t i = 1; i <= count; ++i)
    {
        const double fraction = static_cast<double>(i) / count;
        auto pose = geometry::interpolatePose(current, destination, fraction);
        pose.pos = m_center + m_clearanceRadius * (Eigen::Quaterniond::Identity().slerp(fraction, rotation) * from);
        path.push_back({pose, MotionType::RAPID});
    }
}
bool SurfaceCutGeometry::cuttingEndpointsClear(const std::vector<Move>& path) const
{
    // Reject gouging before validating the more expensive access motion.
    for (size_t i = path.size() - 3; i < path.size() - 1; ++i)
    {
        if (m_scene.model->intersects(assembly(), geometry::toolTransform(path[i].pose)))
        {
            return false;
        }
    }
    return true;
}
}  // namespace transmuter::pathgenerator::algorithms
