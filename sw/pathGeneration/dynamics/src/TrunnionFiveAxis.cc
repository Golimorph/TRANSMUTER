#include "TrunnionFiveAxis.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "trace.h"

namespace transmuter::pathgenerator
{
TrunnionFiveAxis::TrunnionFiveAxis(const MachineConfig& config) : m_config(config)
{
    if (!config.jointPositionInMachine.allFinite() || !config.workpieceCenterFromJoint.allFinite() ||
        !std::isfinite(config.minimumTilt) || !std::isfinite(config.maximumTilt) || config.minimumTilt < -std::numbers::pi || config.maximumTilt > std::numbers::pi ||
        config.minimumTilt > config.maximumTilt)
    {
        ERROR("Invalid trunnion dimensions or tilt limits.");
    }
}
std::vector<geometry::MachineAxes> TrunnionFiveAxis::candidates(const geometry::Vertex& pose, const std::optional<geometry::MachineAxes>& previous) const
{
    std::vector<geometry::MachineAxes> result;
    const double angle = std::acos(std::clamp(pose.normal.normalized().z(), -1.0, 1.0));
    addBranch(result, pose, -angle, previous);
    if (angle > 1e-8)
    {
        addBranch(result, pose, angle, previous);
    }
    return result;
}
void TrunnionFiveAxis::addBranch(std::vector<geometry::MachineAxes>& result, const geometry::Vertex& pose, double tilt, const std::optional<geometry::MachineAxes>& previous) const
{
    if (tilt < m_config.minimumTilt || tilt > m_config.maximumTilt)
    {
        return;
    }
    const Eigen::Vector3d normal = pose.normal.normalized();
    const double lastB = previous ? previous->b : 0.0;
    const double branchB = std::atan2(-normal.y(), normal.x()) + (tilt > 0.0 ? std::numbers::pi : 0.0);
    const double azimuth = normal.head<2>().norm() < 1e-8 ? lastB : lastB + std::remainder(branchB - lastB, 2.0 * std::numbers::pi);
    if (auto axes = resolveBranch(pose, tilt, azimuth))
    {
        result.push_back(*axes);
    }
}
std::array<bool, 5> TrunnionFiveAxis::movableAxes() const
{
    return {true, true, true, true, true};
}
bool TrunnionFiveAxis::hasRedundantMotion() const
{
    return true;
}
std::optional<geometry::MachineAxes> TrunnionFiveAxis::resolveBranch(const geometry::Vertex& pose, double tilt, double azimuth) const
{
    const Eigen::Matrix3d rotation = (Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(azimuth, Eigen::Vector3d::UnitZ()))
                                         .toRotationMatrix();
    const Eigen::Vector3d position = m_config.jointPositionInMachine + rotation * (m_config.workpieceCenterFromJoint + pose.pos);
    return geometry::MachineAxes{-position.x(), -position.y(), position.z(), tilt, azimuth};
}
MachineFrames TrunnionFiveAxis::frames(const geometry::MachineAxes& axes) const
{
    MachineFrames result;
    Eigen::Affine3d table = Eigen::Affine3d::Identity();
    table.translate(Eigen::Vector3d(axes.x, axes.y, 0.0));
    const Eigen::Affine3d tilt = table * Eigen::Translation3d(m_config.jointPositionInMachine) * Eigen::AngleAxisd(axes.a, Eigen::Vector3d::UnitY());
    const Eigen::Affine3d rotary = tilt * Eigen::AngleAxisd(axes.b, Eigen::Vector3d::UnitZ());
    result.workpiece = rotary * Eigen::Translation3d(m_config.workpieceCenterFromJoint);
    result.tool.translate(Eigen::Vector3d(0.0, 0.0, axes.z));
    return result;
}
Eigen::Vector3d TrunnionFiveAxis::linearAxisForce(const Eigen::Vector3d& force, const geometry::MachineAxes& axes) const
{
    const Eigen::Vector3d machineForce = frames(axes).workpiece.linear() * force;
    return {machineForce.x(), machineForce.y(), -machineForce.z()};
}
}  // namespace transmuter::pathgenerator
