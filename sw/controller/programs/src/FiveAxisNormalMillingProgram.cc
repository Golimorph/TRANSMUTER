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
    const auto scene = createMillingScene();
    const auto config = pathConfig();
    visualization::Visualizer visualizer(1024, 760, "Transmuter: five-axis normal milling", scene);
    pathgenerator::PathGenerator pathGenerator(pathgenerator::PathAlgorithmType::FIVE_AXIS_NORMAL, config, scene, stopSignal());
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
geometry::MillingScene FiveAxisNormalMillingProgram::createMillingScene() const
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
    return {
        .stock = stock,
        .model = geometry::GeometryFactory::createTorus(50.0, 20.0, geometry::GeometryRepresentation::POINT_CLOUD_SOLID, 3.0),
        .workpiece = stock->clone(),
        .machinePath = machine->path(),
        .toolModel = geometry::GeometryFactory::createCylinder(10.0, 20.0, geometry::GeometryRepresentation::ANALYTIC),
        .toolAssemblyModel = assembly,
        .machineKinematics = machine,
    };
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
