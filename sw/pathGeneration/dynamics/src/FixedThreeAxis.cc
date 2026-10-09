#include "FixedThreeAxis.h"

#include "TrunnionFiveAxis.h"
namespace transmuter::pathgenerator
{
FixedThreeAxis::FixedThreeAxis(const MachineConfig& config)
    : m_trunnion(std::make_shared<TrunnionFiveAxis>(config))
{
}
std::vector<geometry::MachineAxes> FixedThreeAxis::candidates(const geometry::Vertex& pose, const std::optional<geometry::MachineAxes>&) const
{
    if (!pose.normal.normalized().isApprox(Eigen::Vector3d::UnitZ(), 1e-8))
    {
        return {};
    }
    return m_trunnion->candidates(pose, std::nullopt);
}
std::array<bool, 5> FixedThreeAxis::movableAxes() const
{
    return {true, true, true, false, false};
}
bool FixedThreeAxis::hasRedundantMotion() const
{
    return false;
}
MachineFrames FixedThreeAxis::frames(const geometry::MachineAxes& axes) const
{
    return m_trunnion->frames(axes);
}
Eigen::Vector3d FixedThreeAxis::linearAxisForce(const Eigen::Vector3d& force, const geometry::MachineAxes& axes) const
{
    return m_trunnion->linearAxisForce(force, axes);
}
}  // namespace transmuter::pathgenerator
