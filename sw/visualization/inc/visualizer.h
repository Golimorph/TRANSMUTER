#ifndef TRANSMUTER_VISUALIZATION_VISUALIZER_H
#define TRANSMUTER_VISUALIZATION_VISUALIZER_H

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "geometryIf.h"
#include "millingScene.h"
#include "renderer.h"
#include "toolPathIf.h"

namespace transmuter::visualization
{

struct VisualizerConfig
{
    double toolPathPlaybackDurationSeconds = 60.0;
    // Relative to CUTTING while preserving the configured total playback duration.
    double rapidPlaybackSpeedMultiplier = 5.0;
};

/*!
 * @brief A class for visualizing the milling simulation.
 * Calling the constructor initializes the rendering window.
 * calling refresh updates the scene.
 * Shared scene state lets refresh observe live tool-path and workpiece updates.
 * This class is the main interface for the graphics engine.
 */
class Visualizer
{
   public:
    Visualizer(int w, int h, const std::string& title, const transmuter::geometry::MillingScene& millingScene, VisualizerConfig config = {});

    /*! @brief Refreshes the visualization.
     * @return true if the visualization was updated. False if the window was closed.*/
    bool refresh();

    void startToolPathPlayback();
    bool isToolPathPlaybackFinished() const;

   private:
    struct PlaybackState
    {
        std::chrono::steady_clock::time_point startTime;
        std::vector<geometry::ToolPathPoint> toolPath;
        std::vector<double> cumulativeTravelTimes;
        geometry::Vertex toolPose;
        double pathTravelTime = 0.0;
        size_t processedPointCount = 0;
        bool started = false;
        bool active = false;
        bool finished = false;
    };

    void validateInputs() const;
    void updateToolPathPlayback();
    void calculatePlaybackTimes();
    double calculatePlaybackProgress() const;
    void updateToolPose(double targetTravelTime);
    void removeTraversedMaterial(double targetTravelTime);
    void finishToolPathPlayback();
    geometry::Vertex interpolateToolPose(size_t upperIndex, double travelTime) const;
    geometry::Vertex getToolNode() const;

    Renderer m_renderer;
    std::shared_ptr<const geometry::GeometryIf> p_stock = nullptr;
    std::shared_ptr<const geometry::GeometryIf> p_model = nullptr;
    std::shared_ptr<geometry::GeometryIf> p_workpiece = nullptr;
    std::shared_ptr<geometry::ToolPathIf> p_toolPath = nullptr;
    std::shared_ptr<const geometry::GeometryIf> p_toolModel = nullptr;
    VisualizerConfig m_config;
    PlaybackState m_playback;
};

}  // namespace transmuter::visualization

#endif  // TRANSMUTER_VISUALIZATION_VISUALIZER_H
