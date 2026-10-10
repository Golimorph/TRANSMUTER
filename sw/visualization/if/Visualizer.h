#ifndef TRANSMUTER_VISUALIZATION_VISUALIZER_H
#define TRANSMUTER_VISUALIZATION_VISUALIZER_H

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "MachineKinematicsIf.h"
#include "MachinePathIf.h"
#include "PathGenerationContext.h"
#include "UserMillingRequest.h"
#include "geometryIf.h"
#include "millingScene.h"

namespace transmuter::visualization
{
class Renderer;
class MachinePathPlayback;
class MachineVisualizationIf;

/*!
 * @brief A class for visualizing the milling simulation.
 * Window events, frame rendering and playback run on the caller's thread.
 * This class is the main interface for the graphics engine.
 */
class Visualizer
{
   public:
    struct Config
    {
        int windowWidth = 1024;
        int windowHeight = 760;
        std::string windowTitle = "Transmuter";
        double toolPathPlaybackDurationSeconds = 60.0;
    };

    Visualizer(const transmuter::geometry::MillingScene& millingScene, const pathgenerator::PathGenerationContext& pathGeneration, Config config);
    ~Visualizer();

    /** Process window events and render one frame; false means the window was closed. */
    bool processEvents();
    std::optional<UserMillingRequest> takeMillingRequest();
    /** Show the current operation while it is generated, before it joins the accepted path. */
    void setPreviewMachinePath(std::shared_ptr<const pathgenerator::MachinePathIf> path);
    void startToolPathPlayback();

   private:
    enum class ViewCoordinates
    {
        WORKPIECE,
        MACHINE
    };

    void renderFrame();
    void drawScene();

    void refreshPathSnapshots();
    pathgenerator::MachineMove displayedMachineMove() const;
    std::vector<geometry::Vertex> toolPoses(const std::vector<pathgenerator::MachineMove>& path) const;
    void validateConfig() const;

    std::unique_ptr<Renderer> m_renderer;
    std::shared_ptr<const geometry::GeometryIf> m_stock;
    std::shared_ptr<geometry::GeometryIf> m_workpiece;
    std::shared_ptr<const pathgenerator::MachinePathIf> m_machinePath;
    std::shared_ptr<const geometry::GeometryIf> m_toolAssemblyModel;
    std::shared_ptr<const pathgenerator::MachineKinematicsIf> m_machineKinematics;
    std::vector<pathgenerator::MachineMove> m_combinedPathSnapshot;
    std::vector<pathgenerator::MachineMove> m_previewPathSnapshot;
    std::shared_ptr<const pathgenerator::MachinePathIf> m_previewMachinePath;
    Config m_config;
    ViewCoordinates m_viewCoordinates = ViewCoordinates::WORKPIECE;
    std::unique_ptr<MachinePathPlayback> m_playback;
    std::unique_ptr<MachineVisualizationIf> m_machineVisualization;
};

}  // namespace transmuter::visualization

#endif  // TRANSMUTER_VISUALIZATION_VISUALIZER_H
