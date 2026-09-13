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
    const auto millingScene = createMillingScene();
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
                    millingEnvironment.visualizer.startToolPathPlayback();
                    setState(ProgramState::FINISHED);
                }
                break;
            case ProgramState::FINISHED:
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

    m_stock = transmuter::geometry::shapes::Cylinder::generateSolid(100.0f, 200.0f, 5.0f);

    // The stock remains an immutable reference. Cutting updates only the workpiece snapshot.
    m_workpiece = m_stock->clone();

    Eigen::Vector3f dims(50.0f, 50.0f, 50.0f);
    m_model = transmuter::geometry::shapes::Box::generate(dims, 5.0f);

    m_toolPath = std::make_shared<transmuter::geometry::PointCloud>();

    m_toolModel = transmuter::geometry::shapes::Endmill::generate(10.0f, 30.0f);

    return transmuter::geometry::MillingScene{
        .stock = m_stock,
        .model = m_model,
        .workpiece = m_workpiece,
        .toolPath = m_toolPath,
        .toolModel = m_toolModel,
    };
}

BoxMillingProgram::MillingEnvironment BoxMillingProgram::createMillingEnvironment(
    const transmuter::geometry::MillingScene& millingScene) const
{
    INFO("Creating milling environment.");

    transmuter::pathgenerator::PathGeneratorConfig config{
        .algorithmType = transmuter::pathgenerator::AlgorithmType::THREE_AXIS_SPIRAL,
        .pointSpacing = 2.0,
        .passSpacing = 2.0,
        .layerSpacing = 10.0,
    };
    transmuter::visualization::VisualizerConfig visualizerConfig{
        .toolPathPlaybackDurationSeconds = 30.0,
    };

    return MillingEnvironment{
        .visualizer = transmuter::visualization::Visualizer(
            102, 76, "Transmuter: 5-Axis Path Simulation", millingScene, visualizerConfig),
        .pathGenerator = transmuter::pathgenerator::PathGenerator(config, millingScene)};
}

}  // namespace transmuter::controller::programs
