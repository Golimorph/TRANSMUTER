#ifndef TRANSMUTER_VISUALIZATION_RENDERER_H
#define TRANSMUTER_VISUALIZATION_RENDERER_H

#include <GLFW/glfw3.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "geometryIf.h"
#include "toolPathIf.h"

namespace transmuter::visualization
{

class Renderer
{
   public:
    Renderer(int width, int height, const std::string& title);
    ~Renderer();

    bool shouldClose() const;
    void beginFrame();
    void endFrame();
    void handleScroll(double yoffset);

    void drawWorkpiece(const geometry::GeometryIf& workpiece);
    void drawToolPath(const geometry::ToolPathIf& toolPath);
    void drawTool(const geometry::GeometryIf& toolModel, const Eigen::Vector3d& pos, const Eigen::Vector3d& normal);
    void drawStock(const geometry::GeometryIf& stock);
    void drawModel(const geometry::GeometryIf& model);

   private:
    void handleInput();

    GLFWwindow* m_window = nullptr;
    int m_width;
    int m_height;
    double m_rotX;
    double m_rotY;
    double m_zoom = -300.0;

    double m_lastMouseX = 0.0;
    double m_lastMouseY = 0.0;
    std::vector<geometry::Vertex> m_pointCloudSnapshot;
    std::vector<geometry::ToolPathPoint> m_toolPathSnapshot;
};

}  // namespace transmuter::visualization

#endif
