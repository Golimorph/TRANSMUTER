#include "visualizer.h"

#include <algorithm>
#include <chrono>
#include <iterator>

#include "trace.h"

namespace transmuter::visualization
{

Visualizer::Visualizer(int w, int h, const std::string& title,
                       const transmuter::geometry::MillingScene& millingScene,
                       VisualizerConfig config)
    : m_renderer(w, h, title),
      p_stock(millingScene.stock),
      p_model(millingScene.model),
      p_workpiece(millingScene.workpiece),
      p_toolPath(millingScene.toolPath),
      p_toolModel(millingScene.toolModel),
      m_config(config)
{
    validateInputs();
    m_materialRemoval = geometry::createMaterialRemoval(*p_workpiece, *p_toolModel);

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
}

void Visualizer::startToolPathPlayback()
{
    p_toolPath->copyTo(m_playback.toolPath);
    if (m_playback.toolPath.empty())
    {
        INFO("Visualizer: Cannot start playback because the tool path is empty.");
        return;
    }

    calculatePlaybackDistances();
    m_playback.toolPose = m_playback.toolPath.front();
    m_playback.processedPointCount = 0;
    m_playback.startTime = std::chrono::steady_clock::now();
    m_playback.started = true;
    m_playback.active = true;
    m_playback.finished = false;
    INFO("Visualizer: Starting tool-path playback over "
         << m_config.toolPathPlaybackDurationSeconds << " seconds.");
}

void Visualizer::calculatePlaybackDistances()
{
    m_playback.cumulativeDistances.assign(m_playback.toolPath.size(), 0.0);
    for (size_t index = 1; index < m_playback.toolPath.size(); ++index)
    {
        m_playback.cumulativeDistances[index] =
            m_playback.cumulativeDistances[index - 1] +
            static_cast<double>(
                (m_playback.toolPath[index].pos - m_playback.toolPath[index - 1].pos).norm());
    }
    m_playback.pathLength = m_playback.cumulativeDistances.back();
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
    //m_renderer.drawModel(*p_model);

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
    const double targetDistance = progress * m_playback.pathLength;
    updateToolPose(targetDistance);
    removeTraversedMaterial(targetDistance);

    if (progress >= 1.0)
    {
        finishToolPathPlayback();
    }
}

double Visualizer::calculatePlaybackProgress() const
{
    const double elapsedSeconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - m_playback.startTime)
            .count();
    return std::clamp(elapsedSeconds / m_config.toolPathPlaybackDurationSeconds, 0.0, 1.0);
}

void Visualizer::updateToolPose(double targetDistance)
{
    const auto upper = std::lower_bound(m_playback.cumulativeDistances.begin(),
                                        m_playback.cumulativeDistances.end(), targetDistance);
    const size_t upperIndex = static_cast<size_t>(
        std::distance(m_playback.cumulativeDistances.begin(), upper));

    if (upperIndex >= m_playback.toolPath.size())
    {
        m_playback.toolPose = m_playback.toolPath.back();
    }
    else
    {
        m_playback.toolPose = interpolateToolPose(upperIndex, targetDistance);
    }
}

void Visualizer::removeTraversedMaterial(double targetDistance)
{
    const auto completedEnd = std::upper_bound(m_playback.cumulativeDistances.begin(),
                                               m_playback.cumulativeDistances.end(),
                                               targetDistance);
    const size_t completedPointCount = static_cast<size_t>(
        std::distance(m_playback.cumulativeDistances.begin(), completedEnd));

    std::vector<geometry::Vertex> newToolPoses;
    if (m_playback.processedPointCount < completedPointCount)
    {
        newToolPoses.insert(newToolPoses.end(),
                            m_playback.toolPath.begin() + m_playback.processedPointCount,
                            m_playback.toolPath.begin() + completedPointCount);
        m_playback.processedPointCount = completedPointCount;
    }
    // Include the interpolated pose so material removal progresses smoothly within a path segment.
    newToolPoses.push_back(m_playback.toolPose);
    m_materialRemoval->apply(*p_workpiece, newToolPoses);
}

void Visualizer::finishToolPathPlayback()
{
    m_playback.active = false;
    m_playback.finished = true;
    INFO("Visualizer: Tool-path playback complete.");
}

geometry::Vertex Visualizer::interpolateToolPose(size_t upperIndex, double distance) const
{
    if (upperIndex == 0 || upperIndex >= m_playback.toolPath.size())
    {
        return m_playback.toolPath[std::min(upperIndex, m_playback.toolPath.size() - 1)];
    }

    const size_t lowerIndex = upperIndex - 1;
    const double segmentStart = m_playback.cumulativeDistances[lowerIndex];
    const double segmentLength = m_playback.cumulativeDistances[upperIndex] - segmentStart;
    const float factor = segmentLength > 0.0
                             ? static_cast<float>((distance - segmentStart) / segmentLength)
                             : 0.0f;

    geometry::Vertex result;
    result.pos = m_playback.toolPath[lowerIndex].pos +
                 factor * (m_playback.toolPath[upperIndex].pos -
                           m_playback.toolPath[lowerIndex].pos);
    const Eigen::Vector3f& lower = m_playback.toolPath[upperIndex - 1].normal;
    const Eigen::Vector3f& upper = m_playback.toolPath[upperIndex].normal;
    result.normal = lower + factor * (upper - lower);
    if (!result.normal.isZero())
    {
        result.normal.normalize();
    }
    else
    {
        result.normal = lower.isZero() ? Eigen::Vector3f::UnitZ() : lower.normalized();
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
