#include "boxMillingProgram.h"

#include "box.h"
#include "cylinder.h"
#include "endmill.h"
#include "pathGeneratorConfig.h"
#include "pointCloud.h"
#include "trace.h"

namespace transmuter::controller::programs
{

void BoxMillingProgram::execute()
{
    INFO("Starting execution.");
    auto millingScene = createMillingScene();
    auto millingEnvironment = createMillingEnvironment(millingScene);

    setState(ProgramState::RUNNING);
    INFO("Starting path generation.");
    millingEnvironment.pathGenerator.startGeneration();

    while (millingEnvironment.visualizer.refresh())
    {
        switch (getState())
        {
            case ProgramState::RUNNING:
                if (millingEnvironment.pathGenerator.isGenerationFinished())
                {
                    INFO("Path generation finished.");
                    setState(ProgramState::FINISHED);
                }
                break;
            case ProgramState::FINISHED:
                millingEnvironment.visualizer.advanceTool(0.005);
                break;
            case ProgramState::STOPPED:
            default:
                break;
        }
    }
    if (getState() == ProgramState::RUNNING)
    {
        setState(ProgramState::FINISHED);
    }
    INFO("PathProgram: Execution ended.");
}

transmuter::geometry::MillingScene BoxMillingProgram::createMillingScene()
{
    INFO("Creating milling scene.");

    m_stock = transmuter::geometry::shapes::Cylinder::generate(100.0f, 200.0f, 5.0f);

    m_workPiece = m_stock;

    Eigen::Vector3f dims(50.0f, 50.0f, 50.0f);
    m_model = transmuter::geometry::shapes::Box::generate(dims / 2, 5.0f);

    m_toolPath = std::make_shared<transmuter::geometry::PointCloud>();

    m_toolModel = transmuter::geometry::shapes::Endmill::generate(10.0f, 30.0f);

    return transmuter::geometry::MillingScene{m_stock, m_model, m_workPiece, m_toolPath,
                                              m_toolModel};
}

BoxMillingProgram::MillingEnvironment BoxMillingProgram::createMillingEnvironment(
    const transmuter::geometry::MillingScene& millingScene) const
{
    INFO("Creating milling environment.");

    transmuter::pathgenerator::PathGeneratorConfig config{
        .algorithmType = transmuter::pathgenerator::AlgorithmType::THREE_AXIS_SPIRAL,
        .direction = transmuter::pathgenerator::Direction::FORWARD,
        .toolDiameter = 4.0,
        .stepOver = 2.0,
        .maxDepth = 10.0,
    };

    return MillingEnvironment{
        .visualizer = transmuter::visualization::Visualizer(
            1024, 768, "Transmuter: 5-Axis Path Simulation", millingScene),
        .pathGenerator = transmuter::pathgenerator::PathGenerator(config, millingScene)};
}

}  // namespace transmuter::controller::programs
