#ifndef TRANSMUTER_VISUALIZATION_RENDERER_H
#define TRANSMUTER_VISUALIZATION_RENDERER_H

#include <GLFW/glfw3.h>

#include <Eigen/Dense>
#include <memory>
#include <string>

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

    void drawWorkpiece(std::shared_ptr<geometry::PointCloud> workpiece);
    void drawToolPath(std::shared_ptr<geometry::PointCloud> toolPath);
    void drawTool(std::shared_ptr<geometry::PointCloud> toolModel, const Eigen::Vector3f& pos,
                  const Eigen::Vector3f& normal);
    void drawStock(std::shared_ptr<geometry::PointCloud> stock);
    void drawModel(std::shared_ptr<geometry::PointCloud> model);

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
};

}  // namespace transmuter::visualization

#endif