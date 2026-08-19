#include "boxMillingProgram.h"

#include "box.h"
#include "cylinder.h"
#include "endmill.h"
#include "pathGenerator.h"
#include "pathGeneratorConfig.h"
#include "pointCloud.h"
#include "trace.h"
#include "visualizer.h"

namespace transmuter::controller::programs
{

void BoxMillingProgram::execute()
{
    INFO("BoxMillingProgram: Starting execution.");

    auto millingScene = createMillingScene();
    INFO("BoxMillingProgram: Milling scene created.");

    transmuter::visualization::Visualizer visualizer(
        1024, 768, "Transmuter: 5-Axis Path Simulation", millingScene);

    transmuter::pathgenerator::PathGenerator pathGen;

    transmuter::pathgenerator::PathGeneratorConfig config{
        .algorithmType = transmuter::pathgenerator::AlgorithmType::THREE_AXIS_SPIRAL,
        .direction = transmuter::pathgenerator::Direction::FORWARD,
        .toolDiameter = 4.0,
        .stepOver = 2.0,
        .maxDepth = 10.0,
    };

    setState(ProgramState::PATH_GENERATION);
    while (visualizer.refresh())
    {
        switch (getState())
        {
            case ProgramState::PATH_GENERATION:

                if (pathGen.isGenerationInProgress())
                {
                    break;  // Wait for generation to finish
                }
                else if (pathGen.isGenerationFinished())
                {
                    INFO("Path generation finished. Starting milling simulation.");
                    setState(ProgramState::MILLING_SIMULATION);
                }
                else
                {
                    INFO("Starting path generation...");
                    pathGen.startGeneration(config, millingScene);
                }
                break;
            case ProgramState::MILLING_SIMULATION:
                visualizer.advanceTool(0.005);
                break;
            default:
                break;
        }
    }
    INFO("PathProgram: Execution finished.");
}

transmuter::geometry::MillingScene BoxMillingProgram::createMillingScene()
{
    INFO("Creating milling scene...");

    m_stock = transmuter::geometry::shapes::Cylinder::generate(100.0f, 200.0f, 5.0f);

    m_workPiece = m_stock;

    Eigen::Vector3f dims(50.0f, 50.0f, 50.0f);
    m_model = transmuter::geometry::shapes::Box::generate(dims / 2, 5.0f);

    m_toolPath = std::make_shared<transmuter::geometry::PointCloud>();

    m_toolModel = transmuter::geometry::shapes::Endmill::generate(10.0f, 30.0f);

    return transmuter::geometry::MillingScene{m_stock, m_model, m_workPiece, m_toolPath,
                                              m_toolModel};
}

}  // namespace transmuter::controller::programs