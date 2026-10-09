#include "QuickTorusMillingProgram.h"

#include <utility>

#include "geometryFactory.h"
#include "machineFactory.h"
#include "pathComposition.h"
#include "trace.h"

namespace transmuter::controller::programs
{
void QuickTorusMillingProgram::execute()
{
    m_scene = createMillingScene();
    m_combinedPath = pathgenerator::createMachinePath();
    m_scene.machinePath = m_combinedPath;
    const auto roughConfig = roughingConfig();
    const auto fineConfig = fineCuttingConfig();
    // Stages advance this local stock; playback owns the scene workpiece.
    std::shared_ptr<const geometry::GeometryIf> operationStock = m_scene.stock;
    visualization::Visualizer visualizer(1024, 760, "Transmuter: rough and fine torus milling", m_scene);
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
                startCuttingOperation(pathgenerator::PathAlgorithmType::THREE_AXIS_RASTER, m_scene, roughConfig, visualizer);
                setState(ProgramState::ROUGH_CUTTING);
                break;
            case ProgramState::ROUGH_CUTTING:
            {
                if (!m_pathGenerator->isGenerationFinished())
                {
                    break;
                }
                const auto roughResult = m_pathGenerator->getResult();
                appendOperationToCombinedPath(roughResult, operationStock, roughConfig.pointSpacing);
                visualizer.setPreviewMachinePath({});
                operationStock = roughResult.workpiece;
                auto fineScene = m_scene;
                fineScene.stock = operationStock;
                startCuttingOperation(pathgenerator::PathAlgorithmType::THREE_AXIS_RASTER, fineScene, fineConfig, visualizer);
                setState(ProgramState::FINE_CUTTING);
                break;
            }
            case ProgramState::FINE_CUTTING:
            {
                if (!m_pathGenerator->isGenerationFinished())
                {
                    break;
                }
                const auto fineResult = m_pathGenerator->getResult();
                appendOperationToCombinedPath(fineResult, operationStock, fineConfig.pointSpacing);
                visualizer.setPreviewMachinePath({});
                visualizer.startToolPathPlayback();
                setState(ProgramState::FINISHED);
                break;
            }
            default:
                break;
        }
    }
    if (m_pathGenerator)
    {
        m_pathGenerator->stopGeneration();
    }
}
geometry::MillingScene QuickTorusMillingProgram::createMillingScene() const
{
    const auto stock = geometry::GeometryFactory::createDodecahedron(100.0, geometry::GeometryRepresentation::POINT_CLOUD_SOLID, 3.0);
    pathgenerator::MachineConfig machineConfig;
    machineConfig.jointPositionInMachine = Eigen::Vector3d(0.0, 0.0, 100.0);
    machineConfig.workpieceCenterFromJoint = Eigen::Vector3d(0.0, 0.0, 50.0);
    machineConfig.hasBacklash = {true, true, false, false, false};
    machineConfig.takeupDistance.fill(0.0);
    return {
        .stock = stock,
        .model = geometry::GeometryFactory::createTorus(50.0, 20.0, geometry::GeometryRepresentation::POINT_CLOUD_SOLID, 3.0),
        .workpiece = stock->clone(),
        .toolModel = geometry::GeometryFactory::createCylinder(10.0, 80.0, geometry::GeometryRepresentation::ANALYTIC),
        .machineKinematics = pathgenerator::createFixedThreeAxisMachine(machineConfig, {.sampleSpacing = 5.0}),
    };
}
pathgenerator::PathAlgorithmConfig QuickTorusMillingProgram::fineCuttingConfig() const
{
    return {
        .pointSpacing = 5.0,
        .passSpacing = 10.0,
        .layerSpacing = 10.0,
        .geometryTolerance = 0.01,
    };
}
pathgenerator::PathAlgorithmConfig QuickTorusMillingProgram::roughingConfig() const
{
    return {
        .pointSpacing = 10.0,
        .passSpacing = 20.0,
        .layerSpacing = 40.0,
        .geometryTolerance = 0.01,
    };
}
void QuickTorusMillingProgram::startCuttingOperation(pathgenerator::PathAlgorithmType algorithm, geometry::MillingScene scene, const pathgenerator::PathAlgorithmConfig& config, visualization::Visualizer& visualizer)
{
    INFO("Path stage: point/pass/layer spacing " << config.pointSpacing << "/" << config.passSpacing << "/"
                                                 << config.layerSpacing << " mm.");
    visualizer.setPreviewMachinePath(scene.machineKinematics->path());
    m_pathGenerator = std::make_unique<pathgenerator::PathGenerator>(algorithm, config, scene, stopSignal());
    m_pathGenerator->startGeneration();
}
void QuickTorusMillingProgram::appendOperationToCombinedPath(const pathgenerator::PathGenerationResult& result, std::shared_ptr<const geometry::GeometryIf> currentMaterial, double geometrySampleSpacing)
{
    auto transitionScene = m_scene;
    transitionScene.stock = std::move(currentMaterial);
    pathgenerator::appendThreeAxisOperationPath(*result.machinePath, *m_combinedPath, transitionScene, geometrySampleSpacing, stopSignal());
}

}  // namespace transmuter::controller::programs
