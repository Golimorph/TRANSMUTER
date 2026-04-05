#include "visualizer.h"

#include "trace.h"

namespace transmuter::visualization
{

Visualizer::Visualizer(int w, int h, const std::string& title) : m_renderer(w, h, title)
{
    INFO("Visualizer: Scene Manager initialized.");
}

void Visualizer::showMillingScene(const geometry::PointCloud& part,
                                  const geometry::PointCloud& path)
{
    if (part.size() == 0 && path.size() == 0)
    {
        SHUTDOWN_ERROR("Visualizer: Scene started with zero data points.");
    }

    float rotation = 0.0f;
    INFO("Visualizer: Entering scene loop. Part: " + std::to_string(part.size()) +
         " pts | Path: " + std::to_string(path.size()) + " pts");

    while (!m_renderer.shouldClose())
    {
        m_renderer.beginFrame();

        // Layer 1: The Part (Geometry)
        if (part.size() > 0)
        {
            m_renderer.drawPointCloud(part, rotation);
        }

        // Layer 2: The Toolpath (Kinematics)
        if (path.size() > 0)
        {
            m_renderer.drawToolPath(path, rotation);
        }

        rotation += 0.15f;
        if (rotation >= 360.0f) rotation -= 360.0f;

        m_renderer.endFrame();
    }

    INFO("Visualizer: Scene loop terminated.");
}

}  // namespace transmuter::visualization