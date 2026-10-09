#include "MachineKinematics.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

#include "BacklashState.h"
#include "MachineAxisOptimizer.h"
#include "MachinePath.h"
#include "toolPose.h"
#include "trace.h"

namespace transmuter::pathgenerator
{
MachineKinematics::MachineKinematics(std::shared_ptr<const MachineMechanicsIf> mechanics, std::shared_ptr<const ToolForceIf> toolForce, const MachineConfig& config, MachineMotionConfig motionConfig)
    : m_mechanics(std::move(mechanics)),
      m_toolForce(std::move(toolForce)),
      m_reach(std::max(1.0, config.jointPositionInMachine.norm() + config.workpieceCenterFromJoint.norm())),
      m_config(config),
      m_motionConfig(motionConfig),
      m_path(std::make_shared<MachinePath>())
{
    if (!m_mechanics || !m_toolForce)
    {
        ERROR("Machine setup requires mechanics and a force model.");
    }
    if (!std::isfinite(m_motionConfig.sampleSpacing) || m_motionConfig.sampleSpacing <= 0.0 ||
        !std::isfinite(m_motionConfig.pathTolerance) || m_motionConfig.pathTolerance < 0.0)
    {
        ERROR("Machine motion sampling and tolerance settings are invalid.");
    }
    if (m_config.hasBacklash[3] || m_config.hasBacklash[4])
    {
        ERROR("Rotary-axis backlash is not supported by this motion check.");
    }
    for (double distance : m_config.takeupDistance)
    {
        if (!std::isfinite(distance) || distance < 0.0)
        {
            ERROR("Machine take-up distances must be finite and nonnegative.");
        }
    }
}

MachinePathResult MachineKinematics::propose(const geometry::Vertex& requestedPose, MotionType motionType, const std::atomic<bool>& stop)
{
    const auto preload = BacklashState::fromPath(*m_path, m_config.hasBacklash, m_config.takeupDistance);
    const auto lastAcceptedMove = m_path->last();
    std::vector<geometry::MachineAxes> candidates;
    if (lastAcceptedMove)
    {
        candidates = findNextPositions(requestedPose, lastAcceptedMove->axes);
    }
    else
    {
        candidates = findStartingPositions(requestedPose);
    }
    for (const auto& axes : candidates)
    {
        if (stop.load())
        {
            return {MachinePathStatus::CANCELLED};
        }
        if (acceptsMove(*m_path, requestedPose, axes, motionType, preload, stop))
        {
            MachineMove move{axes, motionType};
            m_path->append(move);
            return {MachinePathStatus::ACCEPTED};
        }
        if (stop.load())
        {
            return {MachinePathStatus::CANCELLED};
        }
    }
    return {candidates.empty() ? MachinePathStatus::UNREACHABLE : MachinePathStatus::UNSAFE};
}

std::shared_ptr<const MachinePathIf> MachineKinematics::path() const
{
    return m_path;
}

void MachineKinematics::clearPath()
{
    m_path->clear();
}

std::shared_ptr<MachineKinematicsIf> MachineKinematics::fork(const MachinePathIf& acceptedPath) const
{
    auto copy = std::make_shared<MachineKinematics>(m_mechanics, m_toolForce, m_config, m_motionConfig);
    std::vector<MachineMove> moves;
    acceptedPath.copyTo(moves);
    for (const auto& move : moves) copy->m_path->append(move);
    return copy;
}

std::vector<geometry::MachineAxes> MachineKinematics::findStartingPositions(const geometry::Vertex& requested) const
{
    return m_mechanics->candidates(requested, std::nullopt);
}

std::vector<geometry::MachineAxes> MachineKinematics::findNextPositions(const geometry::Vertex& requested, const geometry::MachineAxes& lastAcceptedAxes) const
{
    return MachineAxisOptimizer(*m_mechanics, m_reach, 1e-9).alternatives(requested, lastAcceptedAxes);
}

bool MachineKinematics::endpointReachable(const geometry::Vertex& pose, const geometry::MachineAxes& axes) const
{
    if (!geometry::finiteAxes(axes))
    {
        return false;
    }
    const auto reachable = m_mechanics->candidates(pose, axes);
    const auto expected = axisValues(axes);
    return std::any_of(reachable.begin(), reachable.end(),
        [&expected](const auto& candidate)
        {
            const auto actual = axisValues(candidate);
            for (size_t axis = 0; axis < 5; ++axis)
            {
                if (std::abs(actual[axis] - expected[axis]) > 1e-5)
                {
                    return false;
                }
            }
            return true;
        });
}

size_t MachineKinematics::interpolationSamples(const geometry::MachineAxes& from, const geometry::MachineAxes& to, double spacing) const
{
    const double travel = Eigen::Vector3d(to.x - from.x, to.y - from.y, to.z - from.z).norm() + m_reach * (std::abs(to.a - from.a) + std::abs(to.b - from.b));
    if (!std::isfinite(travel) || travel / spacing > 1000000.0)
    {
        return 0;
    }
    return travel > 1e-9 ? std::max<size_t>(2, static_cast<size_t>(std::ceil(travel / spacing))) : 1;
}

bool MachineKinematics::acceptsMove(const MachinePathIf& path, const geometry::Vertex& requested, const geometry::MachineAxes& endpoint, MotionType motionType, BacklashState preload, const std::atomic<bool>& stop) const
{
    if (!endpointReachable(requested, endpoint))
    {
        return false;
    }
    const auto lastMove = path.last();
    if (!lastMove)
    {
        const auto actual = workpieceToolPose(*m_mechanics, endpoint);
        return actual.pos.isApprox(requested.pos, 1e-5) && actual.normal.isApprox(requested.normal.normalized(), 1e-6);
    }
    const auto startAxes = lastMove->axes;
    const size_t samples = interpolationSamples(startAxes, endpoint, m_motionConfig.sampleSpacing);
    if (samples == 0)
    {
        return false;
    }
    const auto startPose = workpieceToolPose(*m_mechanics, startAxes);
    geometry::Vertex prior = startPose;
    for (size_t i = 1; i <= samples; ++i)
    {
        if (stop.load())
        {
            return false;
        }
        const double fraction = static_cast<double>(i) / samples;
        const auto axes = geometry::interpolateAxes(startAxes, endpoint, fraction);
        const auto actual = workpieceToolPose(*m_mechanics, axes);
        const auto expected = geometry::interpolatePose(startPose, requested, fraction);
        if (!acceptsSample(prior, actual, expected, axes, motionType, preload))
        {
            return false;
        }
        prior = actual;
    }
    return true;
}

bool MachineKinematics::acceptsSample(const geometry::Vertex& from, const geometry::Vertex& actual, const geometry::Vertex& requested, const geometry::MachineAxes& axes, MotionType motionType, BacklashState& preload) const
{
    if ((actual.pos - requested.pos).norm() > m_motionConfig.pathTolerance ||
        m_reach * (actual.normal.normalized() - requested.normal.normalized()).norm() >
            m_motionConfig.pathTolerance)
    {
        return false;
    }
    preload.observe(axes, m_config.takeupDistance);
    if (motionType == MotionType::RAPID)
    {
        return true;
    }
    if ((actual.pos - from.pos).squaredNorm() <= 1e-20)
    {
        return false;
    }
    const auto force = m_mechanics->linearAxisForce(m_toolForce->forceOnWorkpiece(from, actual), axes);
    if (!force.allFinite())
    {
        return false;
    }
    for (size_t axis = 0; axis < 3; ++axis)
    {
        if (m_config.hasBacklash[axis] && std::abs(force[axis]) > 1e-6 &&
            (preload.direction(axis) == 0 || force[axis] * preload.direction(axis) > 1e-6))
        {
            return false;
        }
    }
    return true;
}

MachineDisplayPose MachineKinematics::displayPose(const geometry::MachineAxes& axes) const
{
    const auto frames = m_mechanics->frames(axes);
    const Eigen::Affine3d local = frames.workpiece.inverse() * frames.tool;
    return {{local.translation(), local.linear().col(2)}, frames.workpiece, frames.tool};
}

}  // namespace transmuter::pathgenerator
