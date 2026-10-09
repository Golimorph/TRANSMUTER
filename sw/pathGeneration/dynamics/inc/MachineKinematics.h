#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <vector>

#include "MachineConfig.h"
#include "MachineKinematicsIf.h"
#include "MachineMechanicsIf.h"
#include "MachineMotionConfig.h"
#include "ToolForceIf.h"

namespace transmuter::pathgenerator
{
class BacklashState;

class MachineKinematics final : public MachineKinematicsIf
{
   public:
    MachineKinematics(std::shared_ptr<const MachineMechanicsIf> mechanics, std::shared_ptr<const ToolForceIf> toolForce, const MachineConfig& config, MachineMotionConfig motionConfig);
    MachinePathResult propose(const geometry::Vertex& requestedPose, MotionType motionType, const std::atomic<bool>& stop) override;
    std::shared_ptr<const MachinePathIf> path() const override;
    void clearPath() override;
    std::shared_ptr<MachineKinematicsIf> fork(const MachinePathIf& acceptedPath) const override;
    MachineDisplayPose displayPose(const geometry::MachineAxes& axes) const override;

   private:
    std::vector<geometry::MachineAxes> findStartingPositions(const geometry::Vertex& requested) const;
    std::vector<geometry::MachineAxes> findNextPositions(const geometry::Vertex& requested, const geometry::MachineAxes& lastAcceptedAxes) const;
    bool endpointReachable(const geometry::Vertex& pose, const geometry::MachineAxes& axes) const;
    size_t interpolationSamples(const geometry::MachineAxes& from, const geometry::MachineAxes& to, double spacing) const;
    bool acceptsMove(const MachinePathIf& path, const geometry::Vertex& requested, const geometry::MachineAxes& endpoint, MotionType motionType, BacklashState preload, const std::atomic<bool>& stop) const;
    bool acceptsSample(const geometry::Vertex& from, const geometry::Vertex& actual, const geometry::Vertex& requested, const geometry::MachineAxes& axes, MotionType motionType, BacklashState& preload) const;
    std::shared_ptr<const MachineMechanicsIf> m_mechanics;
    std::shared_ptr<const ToolForceIf> m_toolForce;
    double m_reach = 1.0;
    MachineConfig m_config;
    MachineMotionConfig m_motionConfig;
    std::shared_ptr<MachinePathIf> m_path;
};
}  // namespace transmuter::pathgenerator
