#ifndef TRANSMUTER_VISUALIZATION_VISUALIZER_H
#define TRANSMUTER_VISUALIZATION_VISUALIZER_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <string>

#include "pointCloud.h"

namespace transmuter::visualization
{

/**
 * @brief The Industrial Visualizer for the Transmuter project.
 * * This class encapsulates the GLFW window lifecycle and OpenGL
 * rendering pipeline. It allows the core logic to visualize
 * point clouds and toolpaths without exposing raw GL calls
 * to the rest of the system.
 */
class Visualizer
{
   public:
    /**
     * @brief Construct a new Visualizer object.
     * Initializes GLFW and creates an OpenGL context.
     */
    Visualizer(int width, int height, const std::string& title);

    /**
     * @brief Destroy the Visualizer object.
     * Cleans up resources and terminates the GLFW context.
     */
    ~Visualizer();

    /**
     * @brief Checks if the user has requested to close the window.
     */
    bool shouldClose() const;

    /**
     * @brief Prepares the graphics pipeline for a new frame.
     * Clears buffers and sets up the 3D perspective and camera position.
     */
    void beginFrame();

    /**
     * @brief Visualizes a point cloud with normal-based coloring.
     * * @param cloud The geometry to display.
     * @param rotationAngle The current rotation in degrees around the Y-axis.
     */
    void drawPointCloud(const geometry::PointCloud& cloud, float rotationAngle);

    /**
     * @brief Finalizes the frame.
     * Swaps the front and back buffers and polls for window events.
     */
    void endFrame();

   private:
    GLFWwindow* m_window;
    int m_width;
    int m_height;

    // Internal helper to set up the viewing transformation
    void setupCamera();
};

}  // namespace transmuter::visualization

#endif