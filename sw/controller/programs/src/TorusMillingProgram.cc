#include "TorusMillingProgram.h"

#include "PathGenerator.h"
#include "Visualizer.h"
#include "geometryFactory.h"
#include "machineFactory.h"
#include "trace.h"

namespace transmuter::controller::programs
{

void TorusMillingProgram::execute()
{
    INFO("Starting execution.");
    auto context = createMillingContext();
    const auto config = pathConfig();
    visualization::Visualizer::Config viewConfig;
    viewConfig.windowTitle = "Transmuter: torus milling";
    visualization::Visualizer visualizer(context.geometryContext, context.pathContext, viewConfig);
    pathgenerator::PathGenerator pathGenerator(pathgenerator::PathAlgorithmType::THREE_AXIS_RASTER, config,
        context.geometryContext, *context.pathContext.machineSetup(), stopSignal());
    while (true)
    {
        if (!visualizer.processEvents())
        {
            stop();
        }
        if (isStopRequested())
        {
            setState(ProgramState::STOPPED);
            break;
        }
        switch (getState())
        {
            case ProgramState::IDLE:
                INFO("Starting path generation.");
                pathGenerator.startGeneration();
                visualizer.setPreviewMachinePath(pathGenerator.operationPath());
                setState(ProgramState::RUNNING);
                break;
            case ProgramState::RUNNING:
            {
                if (!pathGenerator.isGenerationFinished())
                {
                    break;
                }
                const auto result = pathGenerator.getResult();
                *context.pathContext.machinePath() += *result.machinePath;
                visualizer.setPreviewMachinePath({});
                visualizer.startToolPathPlayback();
                setState(ProgramState::FINISHED);
                break;
            }
            default:
                break;
        }
    }
    pathGenerator.stopGeneration();
}

MillingProgramContext TorusMillingProgram::createMillingContext() const
{
    INFO("Creating milling scene.");

    const auto stock = transmuter::geometry::GeometryFactory::createDodecahedron(100.0, transmuter::geometry::GeometryRepresentation::POINT_CLOUD_SOLID, 3.0);
    const auto workpiece = stock->clone();
    const auto model = transmuter::geometry::GeometryFactory::createTorus(50.0, 20.0, transmuter::geometry::GeometryRepresentation::POINT_CLOUD_SOLID, 3.0);
    const auto toolModel = transmuter::geometry::GeometryFactory::createCylinder(10.0, 80.0, transmuter::geometry::GeometryRepresentation::ANALYTIC);
    pathgenerator::MachineConfig machineConfig;
    machineConfig.jointPositionInMachine = Eigen::Vector3d(0.0, 0.0, 100.0);
    machineConfig.workpieceCenterFromJoint = Eigen::Vector3d(0.0, 0.0, 50.0);
    machineConfig.hasBacklash = {true, true, false, false, false};
    machineConfig.takeupDistance.fill(0.0);

    auto machine = pathgenerator::createFixedThreeAxisMachine(machineConfig);
    return {geometry::MillingScene(stock, model, workpiece, toolModel), pathgenerator::PathGenerationContext(pathgenerator::createMachinePath(), machine)};
}

pathgenerator::PathAlgorithmConfig TorusMillingProgram::pathConfig() const
{
    return {
        .pointSpacing = 0.5,
        .passSpacing = 5,
        .layerSpacing = 5,
        .geometryTolerance = 0.01,
    };
}

}  // namespace transmuter::controller::programs
