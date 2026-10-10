#include "Visualizer.h"

#include <utility>

#include "MachinePathPlayback.h"
#include "MachineVisualizationIf.h"
#include "Renderer.h"
#include "trace.h"

namespace transmuter::visualization
{

Visualizer::Visualizer(const geometry::MillingScene& millingScene, const pathgenerator::PathGenerationContext& pathGeneration, Config config)
    : m_renderer(std::make_unique<Renderer>(config.windowWidth, config.windowHeight, config.windowTitle)),
      m_stock(millingScene.stock()),
      m_workpiece(millingScene.workpiece()),
      m_machinePath(pathGeneration.machinePath()),
      m_toolAssemblyModel(millingScene.toolAssemblyModel() ? millingScene.toolAssemblyModel()
                                                           : millingScene.toolModel()),
      m_machineKinematics(pathGeneration.machineSetup()),
      m_config(config)
{
    validateConfig();
    m_playback = std::make_unique<MachinePathPlayback>(millingScene, pathGeneration, m_config.toolPathPlaybackDurationSeconds);
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
    m_viewCoordinates = m_renderer->machineViewSelected() ? ViewCoordinates::MACHINE : ViewCoordinates::WORKPIECE;
    renderFrame();
    return true;
}

std::optional<UserMillingRequest> Visualizer::takeMillingRequest()
{
    return m_renderer->takeMillingRequest();
}

void Visualizer::setPreviewMachinePath(std::shared_ptr<const pathgenerator::MachinePathIf> path)
{
    m_previewMachinePath = std::move(path);
}

void Visualizer::startToolPathPlayback()
{
    m_playback->start();
}

void Visualizer::renderFrame()
{
    m_playback->update();
    m_renderer->beginFrame();
    drawScene();
    m_renderer->endFrame();
}

void Visualizer::drawScene()
{
    refreshPathSnapshots();
    const auto move = displayedMachineMove();
    const auto display = m_machineKinematics->displayPose(move.axes);
    Eigen::Affine3d workpieceFrame = Eigen::Affine3d::Identity();
    geometry::Vertex toolPose = display.toolInWorkpiece;
    if (m_viewCoordinates == ViewCoordinates::MACHINE)
    {
        workpieceFrame = display.workpiece;
        toolPose = {display.tool.translation(), display.tool.linear().col(2)};
        const auto machineDrawing = m_machineVisualization->getMachineDrawing(move.axes);
        m_renderer->drawMachine(machineDrawing);
    }
    m_renderer->drawStock(*m_stock, workpieceFrame);
    m_renderer->drawWorkpiece(*m_workpiece, workpieceFrame);
    m_renderer->drawUserSelection(workpieceFrame);
    m_renderer->drawToolPath(toolPoses(m_combinedPathSnapshot), workpieceFrame);
    m_renderer->drawToolPath(toolPoses(m_previewPathSnapshot), workpieceFrame);
    if (m_playback->hasStarted() || !m_combinedPathSnapshot.empty() || !m_previewPathSnapshot.empty() || m_viewCoordinates == ViewCoordinates::MACHINE)
    {
        m_renderer->drawTool(*m_toolAssemblyModel, toolPose.pos, toolPose.normal);
    }
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

void Visualizer::validateConfig() const
{
    if (m_config.toolPathPlaybackDurationSeconds <= 0.0)
    {
        ERROR("Tool-path playback duration must be greater than zero.");
    }
}

}  // namespace transmuter::visualization
