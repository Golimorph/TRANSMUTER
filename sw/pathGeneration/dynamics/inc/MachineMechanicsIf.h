#pragma once

#include <Eigen/Geometry>
#include <array>
#include <memory>
#include <optional>
#include <vector>

#include "MachineFrames.h"
#include "machineAxes.h"
#include "vertex.h"

namespace transmuter::pathgenerator
{
using AxisValues = std::array<double, 5>;
/** Private mechanical model; no path algorithm sees its joints or forces. */
class MachineMechanicsIf
{
   public:
    virtual ~MachineMechanicsIf() = default;
    virtual std::vector<geometry::MachineAxes> candidates(const geometry::Vertex& pose, const std::optional<geometry::MachineAxes>& previous) const = 0;
    virtual std::array<bool, 5> movableAxes() const = 0;
    virtual bool hasRedundantMotion() const = 0;
    virtual MachineFrames frames(const geometry::MachineAxes& axes) const = 0;
    virtual Eigen::Vector3d linearAxisForce(const Eigen::Vector3d& forceOnWorkpiece, const geometry::MachineAxes& axes) const = 0;
};
inline geometry::Vertex workpieceToolPose(const MachineMechanicsIf& mechanics, const geometry::MachineAxes& axes)
{
    const auto frames = mechanics.frames(axes);
    const Eigen::Affine3d local = frames.workpiece.inverse() * frames.tool;
    return {local.translation(), local.linear().col(2)};
}
inline AxisValues axisValues(const geometry::MachineAxes& axes)
{
    return {axes.x, axes.y, axes.z, axes.a, axes.b};
}
}  // namespace transmuter::pathgenerator
