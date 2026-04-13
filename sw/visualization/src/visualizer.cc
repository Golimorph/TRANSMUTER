#include "visualizer.h"

#include <GLFW/glfw3.h>

#include "trace.h"

namespace transmuter::visualization
{

Visualizer::Visualizer(int w, int h, const std::string& title,
                       const transmuter::geometry::MillingScene& millingScene)
    : m_renderer(w, h, title),
      p_stock(millingScene.stock),
      p_model(millingScene.model),
      p_workPiece(millingScene.workpiece),
      p_toolPath(millingScene.toolPath),
      p_toolModel(millingScene.toolModel)
{
    if (!p_stock)
    {
        ERROR("stock is null ptr");
    }
    if (!p_model)
    {
        ERROR("model is null ptr");
    }
    if (!p_workPiece)
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

    INFO("Visualizer: Animation engine ready.");
    refresh();
}

void Visualizer::moveTool(double t)
{
    if (t < 0.0 || t >= 1.0)
    {
        ERROR(
            "Invalid tool path dimensionless coordinate parameter for animation."
            "Must be in [0, 1).");
    }
    m_toolPositionIndex = static_cast<uint64_t>(t * p_toolPath->size());
}

void Visualizer::advanceTool(double distance)
{
    if (distance < 0.0)
    {
        ERROR("Invalid tool path distance parameter for animation. Must be non-negative.");
    }

    uint64_t index = m_toolPositionIndex + static_cast<uint64_t>(distance * p_toolPath->size());

    if (index >= p_toolPath->size())
    {
        INFO("Tool has reached the end of the path. Looping back to start.");
    }

    m_toolPositionIndex = index % p_toolPath->size();
}

bool Visualizer::refresh()
{
    if (m_renderer.shouldClose()) return false;

    m_renderer.beginFrame();

    m_renderer.drawStock(p_stock);
    m_renderer.drawModel(p_model);
    m_renderer.drawWorkpiece(p_workPiece);
    m_renderer.drawToolPath(p_toolPath);

    if (!p_toolPath->isEmpty())
    {
        m_renderer.drawTool(p_toolModel, getToolNode().pos, getToolNode().normal);
    }

    m_renderer.endFrame();
    return true;
}

geometry::Vertex Visualizer::getToolNode() const
{
    if (m_toolPositionIndex >= p_toolPath->size())
    {
        ERROR("Tool position index out of bounds. Check animation parameters and tool path size.");
    }
    static std::vector<transmuter::geometry::Vertex> toolPathRenderSnapshot;
    p_toolPath->copyTo(toolPathRenderSnapshot);
    return toolPathRenderSnapshot[m_toolPositionIndex];
}

}  // namespace transmuter::visualization