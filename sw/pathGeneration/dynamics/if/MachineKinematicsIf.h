#pragma once
#include <atomic>
#include <memory>

#include "MachineDisplayPose.h"
#include "MachinePathIf.h"
#include "MachinePathResult.h"

namespace transmuter::pathgenerator
{
/** Machine-independent access to motion planning and display poses. */
class MachineKinematicsIf
{
   public:
    virtual ~MachineKinematicsIf() = default;
    /** Check sampled joint interpolation before recording an accepted move. */
    virtual MachinePathResult propose(const geometry::Vertex& requestedPose, MotionType motionType, const std::atomic<bool>& stop) = 0;
    /** The accepted moves, exposed read-only to algorithms and visualization. */
    virtual std::shared_ptr<const MachinePathIf> path() const = 0;
    virtual void clearPath() = 0;
    /** Independent planner initialized with an accepted path for speculative moves. */
    virtual std::shared_ptr<MachineKinematicsIf> fork(const MachinePathIf& acceptedPath) const = 0;
    /** Forward kinematics for drawing and for deriving tool poses from stored axes. */
    virtual MachineDisplayPose displayPose(const geometry::MachineAxes& axes) const = 0;
};
}  // namespace transmuter::pathgenerator
