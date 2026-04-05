#ifndef TRANSMUTER_VISUALIZATION_VISUALIZER_H
#define TRANSMUTER_VISUALIZATION_VISUALIZER_H

#include "pointCloud.h"
#include "renderer.h"

namespace transmuter::visualization
{

/**
 * @brief High-level Scene Manager.
 * Animates both the Workpiece and the Toolpath.
 */
class Visualizer
{
   public:
    Visualizer(int w, int h, const std::string& title);

    /**
     * @brief Displays the static part and the moving toolpath.
     * Blocks until the window is closed.
     */
    void showMillingScene(const geometry::PointCloud& part, const geometry::PointCloud& path);

   private:
    Renderer m_renderer;
};

}  // namespace transmuter::visualization
#endif