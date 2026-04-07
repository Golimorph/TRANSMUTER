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
        visualizer.advanceTool(0.01);

        /* TODO state machine logic for program phases. This is where you would implement the actual
        milling logic. switch (getState())
        {
            case ProgramState::PATH_GENERATION:
                generateToolPath(); // In a real program, this would be more complex and
        time-consuming INFO("BoxMillingProgram: Path ready. Transitioning to Simulation.");
                setState(ProgramState::IDLE);
                break;

            case ProgramState::MILLING_SIMULATION:
                // The actual "Work" happens here
                visualizer.advanceTool(0.01);

                // Check if we are done to prevent the OOB error
                if (visualizer.isPathComplete())
                {
                    setState(ProgramState::IDLE);
                    INFO("BoxMillingProgram: Simulation complete.");
                }
                break;

            case ProgramState::PAUSED:
                // Do nothing. The loop still runs, so you can
                // still rotate the camera and look at the part.
                break;

            case ProgramState::FINISHED:
                // Stay here. The window remains open until the user closes it.
                break;

            case ProgramState::IDLE:
                //In a real program, this is where you would wait for user input to start the next
        phase. break;

            default:
                break;
        }
        */
    }
    INFO("PathProgram: Execution finished.");
}

transmuter::geometry::MillingScene BoxMillingProgram::createMillingScene()
{
    Eigen::Vector3f dims(100.0f, 100.0f, 100.0f);

    // Assign to the CLASS MEMBERS so they stay alive in memory
    m_workPiece = std::make_shared<transmuter::geometry::PointCloud>(
        transmuter::geometry::shapes::Box::generate(dims, 5.0f));

    m_toolPath = std::make_shared<transmuter::geometry::PointCloud>(generateToolPath());

    m_toolModel = std::make_shared<transmuter::geometry::PointCloud>(
        transmuter::geometry::shapes::Endmill::generate(4.0f, 30.0f));

    // Return the struct using the persistent members
    return transmuter::geometry::MillingScene{m_workPiece, m_toolPath, m_toolModel};
}

}  // namespace transmuter::controller::programs