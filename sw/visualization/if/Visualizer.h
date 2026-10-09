#ifndef TRANSMUTER_VISUALIZATION_VISUALIZER_H
#define TRANSMUTER_VISUALIZATION_VISUALIZER_H

#include <memory>
#include <string>
#include <vector>

#include "MachineKinematicsIf.h"
#include "MachinePathIf.h"
#include "VisualizerConfig.h"
#include "geometryIf.h"
#include "millingScene.h"
#include "viewCoordinates.h"

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
    Visualizer(int w, int h, const std::string& title, const transmuter::geometry::MillingScene& millingScene, VisualizerConfig config = {});
    ~Visualizer();

    /** Process window events and render one frame; false means the window was closed. */
    bool processEvents();

    void startToolPathPlayback();
    /** Show the current operation while it is generated, before it joins the accepted path. */
    void setPreviewMachinePath(std::shared_ptr<const pathgenerator::MachinePathIf> path);

   private:
    void validateInputs(const geometry::MillingScene& scene) const;
    void renderFrame();
    void refreshPathSnapshots();
    void drawScene();
    pathgenerator::MachineMove displayedMachineMove() const;
    std::vector<geometry::Vertex> toolPoses(const std::vector<pathgenerator::MachineMove>& path) const;

    std::unique_ptr<Renderer> m_renderer;
    std::shared_ptr<const geometry::GeometryIf> m_stock;
    std::shared_ptr<geometry::GeometryIf> m_workpiece;
    std::shared_ptr<const pathgenerator::MachinePathIf> m_machinePath;
    std::shared_ptr<const geometry::GeometryIf> m_toolAssemblyModel;
    std::shared_ptr<const pathgenerator::MachineKinematicsIf> m_machineKinematics;
    std::vector<pathgenerator::MachineMove> m_combinedPathSnapshot;
    std::vector<pathgenerator::MachineMove> m_previewPathSnapshot;
    std::shared_ptr<const pathgenerator::MachinePathIf> m_previewMachinePath;
    VisualizerConfig m_config;
    std::unique_ptr<MachinePathPlayback> m_playback;
    std::unique_ptr<MachineVisualizationIf> m_machineVisualization;
};

}  // namespace transmuter::visualization

#endif  // TRANSMUTER_VISUALIZATION_VISUALIZER_H
