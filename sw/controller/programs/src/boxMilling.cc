#include "boxMilling.h"

#include "box.h"
#include "endmill.h"
#include "trace.h"
#include "visualizer.h"

namespace transmuter::controller::programs
{

void BoxMilling::execute()
{
    using namespace transmuter::geometry;
    using namespace transmuter::visualization;

    // 1. Scene Manager Setup
    // Visualizer wraps the Renderer and handles the window lifecycle internally
    Visualizer scene(1024, 768, "Transmuter: 5-Axis Path Simulation");

    // 2. Geometry Generation (The Workpiece)
    Eigen::Vector3f dims(100.0f, 100.0f, 100.0f);
    auto workPiece = shapes::Box::generate(dims, 5.0f);
    INFO("PathProgram: Generated workpiece with " + std::to_string(workPiece.size()) + " points.");

    // 3. Toolpath Generation (The Mockup)
    // For now, we create a simple diagonal path with tilting normals
    PointCloud toolPath;
    toolPath.reserve(500);

    for (int i = 0; i < 500; ++i)
    {
        float t = i / 500.0f;
        Vertex v;
        // Move from corner to corner
        v.pos = Eigen::Vector3f(-60.0f + (120.0f * t), -60.0f + (120.0f * t), 60.0f);

        // Dynamic Normal: Gradually tilt from vertical (0,0,1) to slanted
        v.normal = Eigen::Vector3f(sin(t * M_PI), 0.0f, 1.0f).normalized();

        toolPath.addVertex(v);
    }
    INFO("PathProgram: Generated mockup toolpath.");

    auto toolModel = shapes::Endmill::generate(4.0f, 30.0f);

    // 4. The Hand-off
    // This call blocks until the user closes the window.
    // It encapsulates the beginFrame/endFrame/rotation logic.
    scene.showMillingScene(workPiece, toolPath, toolModel);

    INFO("PathProgram: Execution finished.");
}

}  // namespace transmuter::controller::programs