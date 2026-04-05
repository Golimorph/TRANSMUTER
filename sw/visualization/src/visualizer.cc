#include "visualizer.h"

#include "trace.h"

namespace transmuter::visualization
{

Visualizer::Visualizer(int w, int h, const std::string& title) : m_renderer(w, h, title)
{
    INFO("Visualizer: Scene Manager initialized.");
}

void Visualizer::showMillingScene(const geometry::PointCloud& part,
                                  const geometry::PointCloud& path,
                                  const geometry::PointCloud& tool)
{
    size_t currentStep = 0;
    float rotation = 0.0f;

    INFO("Visualizer: Starting Animated Scene.");

    while (!m_renderer.shouldClose())
    {
        m_renderer.beginFrame();

        // 1. Static Workpiece
        m_renderer.drawPointCloud(part, rotation);

        // 2. Full Toolpath (The "Ghost" trace)
        m_renderer.drawToolPath(path, rotation);

        // 3. Dynamic Tool (The "Actor")
        if (path.size() > 0)
        {
            const auto& node = path.getVertices()[currentStep];
            // We pass the tool model and the current node's transform
            m_renderer.drawTool(tool, node.pos, node.normal, rotation);

            // Advance to next G-code line (looping back at the end)
            currentStep = (currentStep + 1) % path.size();
        }

        rotation += 0.15f;
        m_renderer.endFrame();
    }
}

}  // namespace transmuter::visualization