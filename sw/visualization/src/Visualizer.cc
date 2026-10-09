#include "Visualizer.h"

#include <utility>

#include "MachinePathPlayback.h"
#include "MachineVisualizationIf.h"
#include "Renderer.h"
#include "trace.h"

namespace transmuter::visualization
{

Visualizer::Visualizer(int w, int h, const std::string& title, const transmuter::geometry::MillingScene& millingScene, VisualizerConfig config)
    : m_renderer(std::make_unique<Renderer>(w, h, title)),
      m_stock(millingScene.stock),
      m_workpiece(millingScene.workpiece),
      m_machinePath(millingScene.machinePath),
      m_toolAssemblyModel(millingScene.toolAssemblyModel ? millingScene.toolAssemblyModel
                                                         : millingScene.toolModel),
      m_machineKinematics(millingScene.machineKinematics),
      m_config(config)
{
    validateInputs(millingScene);
    m_playback = std::make_unique<MachinePathPlayback>(millingScene, m_config);

    m_renderer->setViewCoordinates(m_config.initialCoordinates);
    m_machineVisualization = createMachineVisualization(m_machineKinematics,
        {m_stock->bounds().min.z(), m_toolAssemblyModel->bounds().max.z()});
}

Visualizer::~Visualizer() = default;

bool Visualizer::processEvents()
{
    if (!m_renderer->processEvents())
    {
        return false;
    }
    renderFrame();
    return true;
}
void Visualizer::startToolPathPlayback()
{
    m_playback->start();
}
void Visualizer::setPreviewMachinePath(std::shared_ptr<const pathgenerator::MachinePathIf> path)
{
    m_previewMachinePath = std::move(path);
}

void Visualizer::validateInputs(const geometry::MillingScene& scene) const
{
    if (!m_machineKinematics)
    {
        ERROR("Visualizer needs the path generation kinematics model.");
    }
    if (!m_stock || !m_workpiece || !m_machinePath || !scene.toolModel)
    {
        ERROR("Visualization requires stock, workpiece, machine path and tool geometry.");
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

void Visualizer::renderFrame()
{
    m_playback->update();
    m_renderer->beginFrame();
    drawScene();
    m_renderer->endFrame();
}

pathgenerator::MachineMove Visualizer::displayedMachineMove() const
{
    if (m_playback->hasStarted())
    {
        return m_playback->currentMove();
    }
    if (!m_previewPathSnapshot.empty())
    {
        return m_previewPathSnapshot.back();
    }
    if (!m_combinedPathSnapshot.empty())
    {
        return m_combinedPathSnapshot.back();
    }
    return {};
}
std::vector<geometry::Vertex> Visualizer::toolPoses(const std::vector<pathgenerator::MachineMove>& path) const
{
    std::vector<geometry::Vertex> poses;
    poses.reserve(path.size());
    for (const auto& move : path)
    {
        poses.push_back(m_machineKinematics->displayPose(move.axes).toolInWorkpiece);
    }
    return poses;
}
void Visualizer::refreshPathSnapshots()
{
    m_machinePath->copyTo(m_combinedPathSnapshot);
    if (m_previewMachinePath)
    {
        m_previewMachinePath->copyTo(m_previewPathSnapshot);
    }
    else
    {
        m_previewPathSnapshot.clear();
    }
}
void Visualizer::drawScene()
{
    refreshPathSnapshots();
    const auto move = displayedMachineMove();
    Eigen::Affine3d workpieceFrame = Eigen::Affine3d::Identity();
    geometry::Vertex toolPose = m_machineKinematics->displayPose(move.axes).toolInWorkpiece;
    if (m_renderer->viewCoordinates() == ViewCoordinates::MACHINE)
    {
        const auto display = m_machineKinematics->displayPose(move.axes);
        workpieceFrame = display.workpiece;
        toolPose = {display.tool.translation(), display.tool.linear().col(2)};
        const auto machineDrawing = m_machineVisualization->getMachineDrawing(move.axes);
        m_renderer->drawMachine(machineDrawing);
    }
    m_renderer->drawStock(*m_stock, workpieceFrame);
    m_renderer->drawWorkpiece(*m_workpiece, workpieceFrame);
    m_renderer->drawToolPath(toolPoses(m_combinedPathSnapshot), workpieceFrame);
    m_renderer->drawToolPath(toolPoses(m_previewPathSnapshot), workpieceFrame);
    if (m_playback->hasStarted() || !m_combinedPathSnapshot.empty() || !m_previewPathSnapshot.empty() ||
        m_renderer->viewCoordinates() == ViewCoordinates::MACHINE)
    {
        m_renderer->drawTool(*m_toolAssemblyModel, toolPose.pos, toolPose.normal);
    }
}

}  // namespace transmuter::visualization
