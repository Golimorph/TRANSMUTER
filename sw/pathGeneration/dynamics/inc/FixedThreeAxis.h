#pragma once
#include "MachineConfig.h"
#include "MachineMechanicsIf.h"
namespace transmuter::pathgenerator
{
class FixedThreeAxis final : public MachineMechanicsIf
{
   public:
    explicit FixedThreeAxis(const MachineConfig& config);
    std::vector<geometry::MachineAxes> candidates(const geometry::Vertex& pose, const std::optional<geometry::MachineAxes>& previous) const override;
    std::array<bool, 5> movableAxes() const override;
    bool hasRedundantMotion() const override;
    MachineFrames frames(const geometry::MachineAxes& axes) const override;
    Eigen::Vector3d linearAxisForce(const Eigen::Vector3d& force, const geometry::MachineAxes& axes) const override;

   private:
    std::shared_ptr<const MachineMechanicsIf> m_trunnion;
};
}  // namespace transmuter::pathgenerator
