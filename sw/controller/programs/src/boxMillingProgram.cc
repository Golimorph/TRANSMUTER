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

    m_stock = transmuter::geometry::GeometryFactory::createDodecahedron(100.0, transmuter::geometry::GeometryRepresentation::POINT_CLOUD_SOLID, 3.0);
    m_workpiece = m_stock->clone();
    m_model = transmuter::geometry::GeometryFactory::createTorus(50.0, 20.0, transmuter::geometry::GeometryRepresentation::POINT_CLOUD_SOLID, 3.0);
    m_toolPath = transmuter::geometry::GeometryFactory::createToolPath();
    m_toolModel = transmuter::geometry::GeometryFactory::createCylinder(10.0, 30.0, transmuter::geometry::GeometryRepresentation::ANALYTIC);

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
        .passSpacing = 5.0,
        .layerSpacing = 10.0,
        .geometryTolerance = 0.01,
    };
    transmuter::visualization::VisualizerConfig visualizerConfig{
        .toolPathPlaybackDurationSeconds = 30.0,
    };

    return MillingEnvironment{.visualizer = transmuter::visualization::Visualizer(102, 76,
                                  "Transmuter: 5-Axis Path Simulation", millingScene, visualizerConfig),
        .pathGenerator = transmuter::pathgenerator::PathGenerator(
            transmuter::pathgenerator::PathGenerator::AlgorithmType::THREE_AXIS_RASTER, config,
            millingScene)};
}

}  // namespace transmuter::controller::programs
