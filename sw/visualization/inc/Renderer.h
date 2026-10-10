#ifndef TRANSMUTER_VISUALIZATION_RENDERER_H
#define TRANSMUTER_VISUALIZATION_RENDERER_H

#include <GLFW/glfw3.h>

#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "UserMillingRequest.h"
#include "geometryIf.h"
#include "machineDrawing.h"
#include "vertex.h"

namespace transmuter::visualization
{

class Renderer
{
   public:
    Renderer(int width, int height, const std::string& title);
    ~Renderer();

   private:
    friend class Visualizer;

    /** Process window events and input on the window's thread. */
    bool processEvents();
    std::optional<UserMillingRequest> takeMillingRequest();
    bool machineViewSelected() const;
    void setMachineView(bool selected);

    void beginFrame();
    void drawMachine(const MachineDrawing& drawing);
    void drawStock(const geometry::GeometryIf& stock, const Eigen::Affine3d& transform = Eigen::Affine3d::Identity());
    void drawWorkpiece(const geometry::GeometryIf& workpiece, const Eigen::Affine3d& transform = Eigen::Affine3d::Identity());
    void drawUserSelection(const Eigen::Affine3d& workpieceFrame = Eigen::Affine3d::Identity());
    void drawToolPath(const std::vector<geometry::Vertex>& points, const Eigen::Affine3d& transform = Eigen::Affine3d::Identity());
    void drawTool(const geometry::GeometryIf& toolModel, const Eigen::Vector3d& pos, const Eigen::Vector3d& normal);
    void endFrame();

    void handleInput();
    void handleMillingPanelClick(double x, double y);
    void updateSelection(double x, double y);
    void drawMillingPanel();

    static void scrollCallback(GLFWwindow* window, double xOffset, double yOffset);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int modifiers);
    void handleScroll(double yoffset);
    void updateMillingSlider(double x);
    void updateWindowTitle();
    bool screenToWorkpieceRay(double x, double y, Eigen::Vector3d& nearPoint, Eigen::Vector3d& farPoint) const;
    void drawMachinePart(const MachineDrawingPart& part);

    struct CameraState
    {
        bool machineView = false;
        double thetaDegrees = 55.0;
        double phiDegrees = 45.0;
        double zoom = -300.0;
    };

    enum class SelectionDisplay
    {
        NONE,
        BOX,
        STOCK_HIGHLIGHT
    };

    GLFWwindow* m_window = nullptr;
    std::string m_title;
    int m_width;
    int m_height;
    CameraState m_camera;

    bool m_selectingArea = false;
    bool m_selectionDragging = false;
    bool m_leftMouseDown = false;
    bool m_mousePressPending = false;
    bool m_mouseReleasePending = false;
    bool m_hasSelection = false;
    SelectionDisplay m_selectionDisplay = SelectionDisplay::NONE;
    int m_activeSlider = -1;
    double m_lastMouseX = 0.0;
    double m_lastMouseY = 0.0;
    double m_mousePressX = 0.0;
    double m_mousePressY = 0.0;
    double m_selectionStartX = 0.0;
    double m_selectionStartY = 0.0;
    double m_selectionEndX = 0.0;
    double m_selectionEndY = 0.0;
    std::shared_ptr<const geometry::GeometryIf> m_selectionVolume;
    Eigen::Affine3d m_selectionVolumeToWorkpiece = Eigen::Affine3d::Identity();
    pathgenerator::PathAlgorithmType m_selectedAlgorithm;
    pathgenerator::PathAlgorithmConfig m_pathConfig;
    double m_selectionDepth = 200.0;
    std::optional<UserMillingRequest> m_pendingMillingRequest;
    std::vector<geometry::Vertex> m_pointCloudSnapshot;
};

}  // namespace transmuter::visualization

#endif
