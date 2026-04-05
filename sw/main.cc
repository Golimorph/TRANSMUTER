#include <Eigen/Dense>
#include <iostream>

#include "box.h"
#include "pointCloud.h"
#include "trace.h"
#include "visualizer.h"

/**
 * @brief Entry point for The Transmuter.
 *
 * This high-level conductor orchestrates the geometry generation
 * and the visualization lifecycle without being bogged down by
 * OpenGL or GLFW implementation details.
 */
int main()
{
    using namespace transmuter::geometry;
    using namespace transmuter::visualization;

    // 1. Initialize the Visualization Engine
    // This creates the window and sets up the OpenGL context
    Visualizer visualizer(1024, 768, "The Transmuter: Bernardo Alpha");

    // 2. Generate Workpiece Geometry
    // We create a 100x100x100mm cube with 5mm point density
    Eigen::Vector3f cubeDims(100.0f, 100.0f, 100.0f);
    PointCloud workPiece = shapes::Box::generate(cubeDims, 5.0f);

    INFO("Bernardo Engine: Initialized with " + std::to_string(workPiece.size()) +
         " surface points.");

    float rotationAngle = 0.0f;

    // 3. The Main Execution Loop
    // The Visualizer handles the frame lifecycle and camera setup
    while (!visualizer.shouldClose())
    {
        // Prepares the frame (clears buffers, sets projection/camera)
        visualizer.beginFrame();

        // Visualize the workpiece with its normal vectors
        visualizer.drawPointCloud(workPiece, rotationAngle);

        // Update rotation for the inspection animation
        rotationAngle += 0.4f;

        // Finalizes the frame (swaps buffers, polls input)
        visualizer.endFrame();
    }

    return 0;
}