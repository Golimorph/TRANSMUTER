#include "FiveAxisNormalMillingProgram.h"

#include <numbers>

#include "PathGenerator.h"
#include "Visualizer.h"
#include "geometryFactory.h"
#include "machineFactory.h"
#include "trace.h"

namespace transmuter::controller::programs
{
void FiveAxisNormalMillingProgram::execute()
{
    auto context = createMillingContext();
    const auto config = pathConfig();
    visualization::Visualizer::Config viewConfig;
    viewConfig.windowTitle = "Transmuter: five-axis normal milling";
    visualization::Visualizer visualizer(context.geometryContext, context.pathContext, viewConfig);
    pathgenerator::PathGenerator pathGenerator(pathgenerator::PathAlgorithmType::FIVE_AXIS_NORMAL, config,
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
                if (!result.generationNote.empty())
                {
                    INFO(result.generationNote);
                }
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
MillingProgramContext FiveAxisNormalMillingProgram::createMillingContext() const
{
    const auto stock = geometry::GeometryFactory::createDodecahedron(100.0, geometry::GeometryRepresentation::POINT_CLOUD_SOLID, 3.0);
    const auto assembly = geometry::GeometryFactory::createCylinder(10.0, 80.0, geometry::GeometryRepresentation::ANALYTIC);
    pathgenerator::MachineConfig machineConfig;
    machineConfig.jointPositionInMachine = Eigen::Vector3d(0.0, 0.0, 100.0);
    machineConfig.workpieceCenterFromJoint = Eigen::Vector3d(0.0, 0.0, 50.0);
    machineConfig.minimumTilt = -std::numbers::pi;
    machineConfig.maximumTilt = std::numbers::pi;
    machineConfig.hasBacklash = {true, true, false, false, false};
    machineConfig.takeupDistance.fill(0.0);
    auto machine = pathgenerator::createTrunnionFiveAxisMachine(machineConfig);
    const auto model = geometry::GeometryFactory::createTorus(50.0, 20.0, geometry::GeometryRepresentation::POINT_CLOUD_SOLID, 3.0);
    const auto tool = geometry::GeometryFactory::createCylinder(10.0, 20.0, geometry::GeometryRepresentation::ANALYTIC);
    return {geometry::MillingScene(stock, model, stock->clone(), tool, assembly), pathgenerator::PathGenerationContext(pathgenerator::createMachinePath(), machine)};
}
pathgenerator::PathAlgorithmConfig FiveAxisNormalMillingProgram::pathConfig() const
{
    return {
        .pointSpacing = 1.0,
        .passSpacing = 10.0,
        .layerSpacing = 3.0,
        .geometryTolerance = 0.01,
        .maximumSurfaceLayers = 40,
    };
}
}  // namespace transmuter::controller::programs
