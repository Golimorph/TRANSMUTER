#pragma once
#include "MachineConfig.h"
#include "MachineMechanicsIf.h"
namespace transmuter::pathgenerator
{
class TrunnionFiveAxis final : public MachineMechanicsIf
{
   public:
    explicit TrunnionFiveAxis(const MachineConfig& config);
    std::vector<geometry::MachineAxes> candidates(const geometry::Vertex& pose, const std::optional<geometry::MachineAxes>& previous) const override;
    std::array<bool, 5> movableAxes() const override;
    bool hasRedundantMotion() const override;
    MachineFrames frames(const geometry::MachineAxes& axes) const override;
    Eigen::Vector3d linearAxisForce(const Eigen::Vector3d& force, const geometry::MachineAxes& axes) const override;

   private:
    std::optional<geometry::MachineAxes> resolveBranch(const geometry::Vertex& pose, double tilt, double azimuth) const;
    void addBranch(std::vector<geometry::MachineAxes>& result, const geometry::Vertex& pose, double tilt, const std::optional<geometry::MachineAxes>& previous) const;
    MachineConfig m_config;
};
}  // namespace transmuter::pathgenerator
