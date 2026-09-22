#include "visualizer.h"

#include <algorithm>
#include <chrono>
#include <iterator>

#include "trace.h"

namespace transmuter::visualization
{

namespace
{

Eigen::Affine3d toolTransform(const geometry::Vertex& pose)
{
    if (pose.normal.isZero())
    {
        ERROR("A tool pose must have a non-zero direction.");
    }

    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), pose.normal.normalized())
                             .toRotationMatrix();
    transform.translation() = pose.pos;
    return transform;
}

double relativePlaybackSpeed(
    geometry::MotionSpeed speed, double rapidPlaybackSpeedMultiplier)
{
    switch (speed)
    {
        case geometry::MotionSpeed::CUTTING:
            return 1.0;
        case geometry::MotionSpeed::RAPID:
            return rapidPlaybackSpeedMultiplier;
        default:
            ERROR("Unknown tool-path motion speed.");
    }
    return 1.0;
}

}  // namespace

Visualizer::Visualizer(int w, int h, const std::string& title, const transmuter::geometry::MillingScene& millingScene, VisualizerConfig config)
    : m_renderer(w, h, title),
      p_stock(millingScene.stock),
      p_model(millingScene.model),
      p_workpiece(millingScene.workpiece),
      p_toolPath(millingScene.toolPath),
      p_toolModel(millingScene.toolModel),
      m_config(config)
{
    validateInputs();

    INFO("Visualizer: Animation engine ready.");
    refresh();
}

void Visualizer::validateInputs() const
{
    if (!p_stock)
    {
        ERROR("stock is null ptr");
    }
    if (!p_model)
    {
        ERROR("model is null ptr");
    }
    if (!p_workpiece)
    {
        ERROR("workpiece is null ptr");
    }
    if (!p_toolPath)
    {
        ERROR("toolPath is null ptr");
    }
    if (!p_toolModel)
    {
        ERROR("toolModel is null ptr");
    }
    if (m_config.toolPathPlaybackDurationSeconds <= 0.0)
    {
        ERROR("Tool-path playback duration must be greater than zero.");
    }
    if (m_config.rapidPlaybackSpeedMultiplier <= 0.0)
    {
        ERROR("Rapid playback speed multiplier must be greater than zero.");
    }
}

void Visualizer::startToolPathPlayback()
{
    p_toolPath->copyTo(m_playback.toolPath);
    if (m_playback.toolPath.empty())
    {
        INFO("Visualizer: Cannot start playback because the tool path is empty.");
        return;
    }

    calculatePlaybackTimes();
    m_playback.toolPose = m_playback.toolPath.front().pose;
    m_playback.processedPointCount = 0;
    m_playback.startTime = std::chrono::steady_clock::now();
    m_playback.started = true;
    m_playback.active = true;
    m_playback.finished = false;
    INFO("Visualizer: Starting tool-path playback over " << m_config.toolPathPlaybackDurationSeconds << " seconds.");
}

void Visualizer::calculatePlaybackTimes()
{
    m_playback.cumulativeTravelTimes.assign(m_playback.toolPath.size(), 0.0);
    for (size_t index = 1; index < m_playback.toolPath.size(); ++index)
    {
        const double segmentLength =
            (m_playback.toolPath[index].pose.pos - m_playback.toolPath[index - 1].pose.pos)
                .norm();
        const double segmentSpeed = relativePlaybackSpeed(
            m_playback.toolPath[index].incomingSpeed,
            m_config.rapidPlaybackSpeedMultiplier);
        m_playback.cumulativeTravelTimes[index] =
            m_playback.cumulativeTravelTimes[index - 1] + segmentLength / segmentSpeed;
    }
    m_playback.pathTravelTime = m_playback.cumulativeTravelTimes.back();
}

bool Visualizer::isToolPathPlaybackFinished() const
{
    return m_playback.finished;
}

bool Visualizer::refresh()
{
    if (m_renderer.shouldClose()) return false;

    updateToolPathPlayback();

    m_renderer.beginFrame();

    m_renderer.drawStock(*p_stock);
    m_renderer.drawWorkpiece(*p_workpiece);
    m_renderer.drawToolPath(*p_toolPath);
    // m_renderer.drawModel(*p_model);

    if (m_playback.started && !p_toolPath->isEmpty())
    {
        const geometry::Vertex toolNode = getToolNode();
        m_renderer.drawTool(*p_toolModel, toolNode.pos, toolNode.normal);
    }

    m_renderer.endFrame();
    return true;
}

void Visualizer::updateToolPathPlayback()
{
    if (!m_playback.active || m_playback.toolPath.empty())
    {
        return;
    }

    const double progress = calculatePlaybackProgress();
    const double targetTravelTime = progress * m_playback.pathTravelTime;
    updateToolPose(targetTravelTime);
    removeTraversedMaterial(targetTravelTime);

    if (progress >= 1.0)
    {
        finishToolPathPlayback();
    }
}

double Visualizer::calculatePlaybackProgress() const
{
    const double elapsedSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                                m_playback.startTime)
                                      .count();
    return std::clamp(elapsedSeconds / m_config.toolPathPlaybackDurationSeconds, 0.0, 1.0);
}

void Visualizer::updateToolPose(double targetTravelTime)
{
    const auto upper = std::lower_bound(m_playback.cumulativeTravelTimes.begin(),
        m_playback.cumulativeTravelTimes.end(), targetTravelTime);
    const size_t upperIndex = static_cast<size_t>(
        std::distance(m_playback.cumulativeTravelTimes.begin(), upper));

    if (upperIndex >= m_playback.toolPath.size())
    {
        m_playback.toolPose = m_playback.toolPath.back().pose;
    }
    else
    {
        m_playback.toolPose = interpolateToolPose(upperIndex, targetTravelTime);
    }
}

void Visualizer::removeTraversedMaterial(double targetTravelTime)
{
    const auto completedEnd = std::upper_bound(m_playback.cumulativeTravelTimes.begin(),
        m_playback.cumulativeTravelTimes.end(), targetTravelTime);
    const size_t completedPointCount = static_cast<size_t>(
        std::distance(m_playback.cumulativeTravelTimes.begin(), completedEnd));

    std::vector<geometry::Vertex> newToolPoses;
    for (size_t index = m_playback.processedPointCount; index < completedPointCount; ++index)
    {
        newToolPoses.push_back(m_playback.toolPath[index].pose);
    }
    m_playback.processedPointCount = completedPointCount;
    // Include the interpolated pose so material removal progresses smoothly within a path segment.
    newToolPoses.push_back(m_playback.toolPose);
    std::vector<Eigen::Affine3d> toolTransforms;
    toolTransforms.reserve(newToolPoses.size());
    for (const auto& pose : newToolPoses)
    {
        toolTransforms.push_back(toolTransform(pose));
    }
    p_workpiece->cut(*p_toolModel, toolTransforms);
}

void Visualizer::finishToolPathPlayback()
{
    m_playback.active = false;
    m_playback.finished = true;
    INFO("Visualizer: Tool-path playback complete.");
}

geometry::Vertex Visualizer::interpolateToolPose(
    size_t upperIndex, double travelTime) const
{
    if (upperIndex == 0 || upperIndex >= m_playback.toolPath.size())
    {
        return m_playback.toolPath[std::min(upperIndex, m_playback.toolPath.size() - 1)].pose;
    }

    const size_t lowerIndex = upperIndex - 1;
    const double segmentStart = m_playback.cumulativeTravelTimes[lowerIndex];
    const double segmentDuration =
        m_playback.cumulativeTravelTimes[upperIndex] - segmentStart;
    const double factor =
        segmentDuration > 0.0 ? (travelTime - segmentStart) / segmentDuration : 0.0;

    geometry::Vertex result;
    result.pos = m_playback.toolPath[lowerIndex].pose.pos +
                 factor * (m_playback.toolPath[upperIndex].pose.pos -
                              m_playback.toolPath[lowerIndex].pose.pos);
    const Eigen::Vector3d& lower = m_playback.toolPath[lowerIndex].pose.normal;
    const Eigen::Vector3d& upper = m_playback.toolPath[upperIndex].pose.normal;
    result.normal = lower + factor * (upper - lower);
    if (!result.normal.isZero())
    {
        result.normal.normalize();
    }
    else
    {
        result.normal = lower.isZero() ? Eigen::Vector3d::UnitZ() : lower.normalized();
    }
    return result;
}

geometry::Vertex Visualizer::getToolNode() const
{
    if (!m_playback.started)
    {
        ERROR("Tool pose requested before tool-path playback started.");
    }
    return m_playback.toolPose;
}

}  // namespace transmuter::visualization
