#include "boxMillingProgram.h"

#include "box.h"
#include "endmill.h"
#include "pointCloud.h"
#include "trace.h"
#include "visualizer.h"

namespace transmuter::controller::programs
{

transmuter::geometry::PointCloud generateToolPath()
{
    transmuter::geometry::PointCloud toolPath;
    toolPath.reserve(500);

    for (int i = 0; i < 500; ++i)
    {
        float t = i / 500.0f;
        transmuter::geometry::Vertex v;
        // Move from corner to corner
        v.pos = Eigen::Vector3f(-60.0f + (120.0f * t), -60.0f + (120.0f * t), 60.0f);

        // Dynamic Normal: Gradually tilt from vertical (0,0,1) to slanted
        v.normal = Eigen::Vector3f(sin(t * M_PI), 0.0f, 1.0f).normalized();

        toolPath.addVertex(v);
    }
    return toolPath;
}

void BoxMillingProgram::execute()
{
    INFO("BoxMillingProgram: Starting execution...");
    auto millingScene = createMillingScene();
    INFO("BoxMillingProgram: Milling scene created.");

    transmuter::visualization::Visualizer visualizer(
        1024, 768, "Transmuter: 5-Axis Path Simulation", millingScene);

    while (visualizer.refresh())
    {
        visualizer.advanceTool(0.01);  // Move a bit each frame
    }

    INFO("PathProgram: Execution finished.");
}

transmuter::geometry::MillingScene BoxMillingProgram::createMillingScene() const
{
    Eigen::Vector3f dims(100.0f, 100.0f, 100.0f);
    auto workPiece = std::make_shared<transmuter::geometry::PointCloud>(
        transmuter::geometry::shapes::Box::generate(dims, 5.0f));
    INFO("PathProgram: Generated workpiece with " + std::to_string(workPiece->size()) + " points.");

    auto toolPath = std::make_shared<transmuter::geometry::PointCloud>(generateToolPath());
    INFO("PathProgram: Generated mockup toolpath.");

    auto toolModel = std::make_shared<transmuter::geometry::PointCloud>(
        transmuter::geometry::shapes::Endmill::generate(4.0f, 30.0f));

    return transmuter::geometry::MillingScene{workPiece, toolPath, toolModel};
}

}  // namespace transmuter::controller::programs