#include "MachinePathPlayback.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>

#include "toolPose.h"
#include "trace.h"

namespace transmuter::visualization
{
namespace
{

double relativePlaybackSpeed(pathgenerator::MotionType motionType, double rapidPlaybackSpeedMultiplier)
{
    switch (motionType)
    {
        case pathgenerator::MotionType::CUTTING:
            return 1.0;
        case pathgenerator::MotionType::RAPID:
            return rapidPlaybackSpeedMultiplier;
        default:
            ERROR("Unknown tool-path motion type.");
    }
    return 1.0;
}

}  // namespace

MachinePathPlayback::MachinePathPlayback(const geometry::MillingScene& scene, const VisualizerConfig& config)
    : m_workpiece(scene.workpiece),
      m_sourcePath(scene.machinePath),
      m_tool(scene.toolModel),
      m_assembly(scene.toolAssemblyModel ? scene.toolAssemblyModel : scene.toolModel),
      m_machineKinematics(scene.machineKinematics),
      m_config(config)
{
}
bool MachinePathPlayback::hasStarted() const
{
    return m_started;
}
const pathgenerator::MachineMove& MachinePathPlayback::currentMove() const
{
    return m_currentMove;
}
void MachinePathPlayback::start()
{
    m_sourcePath->copyTo(m_moves);
    if (m_moves.empty())
    {
        INFO("Visualizer: Cannot start playback because the tool path is empty.");
        return;
    }

    calculatePlaybackTimes();
    m_currentMove = m_moves.front();
    m_processedPointCount = 0;
    m_startTime = std::chrono::steady_clock::now();
    m_started = true;
    m_active = true;
    INFO("Visualizer: Starting tool-path playback over " << m_config.toolPathPlaybackDurationSeconds << " seconds.");
}
void MachinePathPlayback::calculatePlaybackTimes()
{
    m_cumulativeTravelTimes.assign(m_moves.size(), 0.0);
    const auto toolBounds = m_assembly->bounds();
    const double toolReach = toolBounds.min.cwiseAbs().cwiseMax(toolBounds.max.cwiseAbs()).norm();
    for (size_t index = 1; index < m_moves.size(); ++index)
    {
        const double segmentLength = segmentTravel(m_moves[index - 1], m_moves[index], toolReach);
        const double segmentSpeed = relativePlaybackSpeed(m_moves[index].motionType, m_config.rapidPlaybackSpeedMultiplier);
        m_cumulativeTravelTimes[index] = m_cumulativeTravelTimes[index - 1] + segmentLength / segmentSpeed;
    }
    m_pathTravelTime = m_cumulativeTravelTimes.back();
}
void MachinePathPlayback::update()
{
    if (!m_active || m_moves.empty())
    {
        return;
    }

    const double progress = calculatePlaybackProgress();
    const double targetTravelTime = progress * m_pathTravelTime;
    updateMove(targetTravelTime);
    removeTraversedMaterial(targetTravelTime);

    if (progress >= 1.0)
    {
        finishPlayback();
    }
}
double MachinePathPlayback::calculatePlaybackProgress() const
{
    const double elapsedSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - m_startTime)
                                      .count();
    return std::clamp(elapsedSeconds / m_config.toolPathPlaybackDurationSeconds, 0.0, 1.0);
}
void MachinePathPlayback::updateMove(double targetTravelTime)
{
    const auto upper = std::lower_bound(m_cumulativeTravelTimes.begin(), m_cumulativeTravelTimes.end(), targetTravelTime);
    const size_t upperIndex = static_cast<size_t>(std::distance(m_cumulativeTravelTimes.begin(), upper));

    if (upperIndex >= m_moves.size())
    {
        m_currentMove = m_moves.back();
    }
    else
    {
        m_currentMove = interpolateMove(upperIndex, targetTravelTime);
    }
}
void MachinePathPlayback::removeTraversedMaterial(double targetTravelTime)
{
    const auto completedEnd = std::upper_bound(m_cumulativeTravelTimes.begin(),
        m_cumulativeTravelTimes.end(), targetTravelTime);
    const size_t completedPointCount = static_cast<size_t>(std::distance(m_cumulativeTravelTimes.begin(), completedEnd));

    std::vector<Eigen::Affine3d> toolTransforms;
    for (size_t index = m_processedPointCount; index < completedPointCount; ++index)
    {
        if (m_moves[index].motionType == pathgenerator::MotionType::CUTTING)
        {
            const auto pose = m_machineKinematics->displayPose(m_moves[index].axes).toolInWorkpiece;
            toolTransforms.push_back(geometry::toolTransform(pose));
        }
    }
    m_processedPointCount = completedPointCount;
    if (m_currentMove.motionType == pathgenerator::MotionType::CUTTING)
    {
        const auto pose = m_machineKinematics->displayPose(m_currentMove.axes).toolInWorkpiece;
        toolTransforms.push_back(geometry::toolTransform(pose));
    }
    if (!toolTransforms.empty())
    {
        m_workpiece->cut(*m_tool, toolTransforms);
    }
}
void MachinePathPlayback::finishPlayback()
{
    m_active = false;
    INFO("Visualizer: Tool-path playback complete.");
}
pathgenerator::MachineMove MachinePathPlayback::interpolateMove(size_t upperIndex, double travelTime) const
{
    if (upperIndex == 0 || upperIndex >= m_moves.size())
    {
        return m_moves[std::min(upperIndex, m_moves.size() - 1)];
    }

    const size_t lowerIndex = upperIndex - 1;
    const double segmentStart = m_cumulativeTravelTimes[lowerIndex];
    const double segmentDuration = m_cumulativeTravelTimes[upperIndex] - segmentStart;
    const double factor = segmentDuration > 0.0 ? (travelTime - segmentStart) / segmentDuration : 0.0;

    return {geometry::interpolateAxes(m_moves[lowerIndex].axes, m_moves[upperIndex].axes, factor),
        m_moves[upperIndex].motionType};
}
double MachinePathPlayback::segmentTravel(const pathgenerator::MachineMove& from, const pathgenerator::MachineMove& to, double reach) const
{
    const auto& a = from.axes;
    const auto& b = to.axes;
    return Eigen::Vector3d(b.x - a.x, b.y - a.y, b.z - a.z).norm() + reach * (std::abs(b.a - a.a) + std::abs(b.b - a.b));
}
}  // namespace transmuter::visualization
