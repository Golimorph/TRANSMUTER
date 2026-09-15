#include "boxMillingProgram.h"

#include "geometryFactory.h"
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

    m_stock = transmuter::geometry::GeometryFactory::createCylinder(
        100.0, 200.0, 5.0, transmuter::geometry::ShapeSampling::SOLID);

    // The stock remains an immutable reference. Cutting updates only the workpiece snapshot.
    m_workpiece = m_stock->clone();

    Eigen::Vector3d dims(50.0, 50.0, 50.0);
    m_model = transmuter::geometry::GeometryFactory::createBox(
        dims, 5.0, transmuter::geometry::ShapeSampling::SOLID);

    m_toolPath = transmuter::geometry::GeometryFactory::createToolPath();

    m_toolModel = transmuter::geometry::GeometryFactory::createEndmill(10.0, 30.0);

    return transmuter::geometry::MillingScene{
        .stock = m_stock,
        .model = m_model,
        .workpiece = m_workpiece,
        .toolPath = m_toolPath,
        .toolModel = m_toolModel,
    };
}

BoxMillingProgram::MillingEnvironment BoxMillingProgram::createMillingEnvironment(const transmuter::geometry::MillingScene& millingScene) const
{
    INFO("Creating milling environment.");

    transmuter::pathgenerator::PathGeneratorConfig config{
        .pointSpacing = 2.0,
        .passSpacing = 2.0,
        .layerSpacing = 10.0,
        .geometryTolerance = 0.01,
    };
    transmuter::visualization::VisualizerConfig visualizerConfig{
        .toolPathPlaybackDurationSeconds = 30.0,
    };

    return MillingEnvironment{.visualizer = transmuter::visualization::Visualizer(102, 76,
                                  "Transmuter: 5-Axis Path Simulation", millingScene, visualizerConfig),
        .pathGenerator = transmuter::pathgenerator::PathGenerator(
            transmuter::pathgenerator::PathGenerator::AlgorithmType::THREE_AXIS_SPIRAL, config,
            millingScene)};
}

}  // namespace transmuter::controller::programs
