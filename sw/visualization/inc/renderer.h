#ifndef TRANSMUTER_VISUALIZATION_RENDERER_H
#define TRANSMUTER_VISUALIZATION_RENDERER_H

#include <GLFW/glfw3.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "pointCloud.h"

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

    void drawWorkpiece(const geometry::PointCloud& workpiece);
    void drawToolPath(const geometry::PointCloud& toolPath);
    void drawTool(const geometry::PointCloud& toolModel, const Eigen::Vector3f& pos,
                  const Eigen::Vector3f& normal);
    void drawStock(const geometry::PointCloud& stock);
    void drawModel(const geometry::PointCloud& model);

   private:
    void handleInput();

    GLFWwindow* m_window;
    int m_width;
    int m_height;
    float m_rotX;
    float m_rotY;
    float m_zoom = -300.0f;

    double m_lastMouseX;
    double m_lastMouseY;
    std::vector<geometry::Vertex> m_pointCloudSnapshot;
};

}  // namespace transmuter::visualization

#endif
