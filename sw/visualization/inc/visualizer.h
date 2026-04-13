#ifndef TRANSMUTER_VISUALIZATION_VISUALIZER_H
#define TRANSMUTER_VISUALIZATION_VISUALIZER_H

#include <memory>
#include <string>
#include <vector>

#include "millingScene.h"
#include "pointCloud.h"
#include "renderer.h"

namespace transmuter::visualization
{
/*!
 * @brief A class for visualizing the milling simulation.
 * Calling the constructor initializes the rendering window.
 * calling refresh updates the scene.
 * the workpiece, toolpath, and tool model are passed as pointers to allow for dynamic updates.
 * This class is the main interface for the graphics engine.
 */
class Visualizer
{
   public:
    Visualizer(int w, int h, const std::string& title,
               const transmuter::geometry::MillingScene& millingScene);

    /*! @brief Refreshes the visualization.
     * @return true if the visualization was updated. False if the window was closed.*/
    bool refresh();

    void moveTool(double position);
    void advanceTool(double distance);

   private:
    geometry::Vertex getToolNode() const;

    Renderer m_renderer;
    std::shared_ptr<geometry::PointCloud> p_stock = nullptr;
    std::shared_ptr<geometry::PointCloud> p_model = nullptr;
    std::shared_ptr<geometry::PointCloud> p_workPiece = nullptr;
    std::shared_ptr<geometry::PointCloud> p_toolPath = nullptr;
    std::shared_ptr<geometry::PointCloud> p_toolModel = nullptr;
    uint64_t m_toolPositionIndex = 0;
};

}  // namespace transmuter::visualization

#endif  // TRANSMUTER_VISUALIZATION_VISUALIZER_H