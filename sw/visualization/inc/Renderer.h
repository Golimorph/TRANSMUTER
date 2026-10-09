#ifndef TRANSMUTER_VISUALIZATION_RENDERER_H
#define TRANSMUTER_VISUALIZATION_RENDERER_H

#include <GLFW/glfw3.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "geometryIf.h"
#include "machineDrawing.h"
#include "vertex.h"
#include "viewCoordinates.h"

namespace transmuter::visualization
{

class Renderer
{
   public:
    Renderer(int width, int height, const std::string& title);
    ~Renderer();

    /** Process window events and input on the window's thread. */
    bool processEvents();
    void beginFrame();
    void endFrame();
    void handleScroll(double yoffset);
    ViewCoordinates viewCoordinates() const;
    void setViewCoordinates(ViewCoordinates coordinates);
    void drawMachine(const MachineDrawing& drawing);

    void drawWorkpiece(const geometry::GeometryIf& workpiece, const Eigen::Affine3d& transform = Eigen::Affine3d::Identity());
    void drawToolPath(const std::vector<geometry::Vertex>& points, const Eigen::Affine3d& transform = Eigen::Affine3d::Identity());
    void drawTool(const geometry::GeometryIf& toolModel, const Eigen::Vector3d& pos, const Eigen::Vector3d& normal);
    void drawStock(const geometry::GeometryIf& stock, const Eigen::Affine3d& transform = Eigen::Affine3d::Identity());
    void drawModel(const geometry::GeometryIf& model);

   private:
    struct CameraState
    {
        ViewCoordinates coordinates = ViewCoordinates::WORKPIECE;
        double thetaDegrees = 55.0;
        double phiDegrees = 45.0;
        double zoom = -300.0;
    };

    void handleInput();
    void drawMachinePart(const MachineDrawingPart& part);

    GLFWwindow* m_window = nullptr;
    std::string m_title;
    int m_width;
    int m_height;
    CameraState m_camera;

    bool m_dragging = false;
    double m_lastMouseX = 0.0;
    double m_lastMouseY = 0.0;
    std::vector<geometry::Vertex> m_pointCloudSnapshot;
};

}  // namespace transmuter::visualization

#endif
