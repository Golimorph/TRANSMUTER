#include "InteractiveTorusMillingProgram.h"

#include <numbers>

#include "geometryFactory.h"
#include "machineFactory.h"
#include "trace.h"

namespace transmuter::controller::programs
{

void InteractiveTorusMillingProgram::execute()
{
    initializeSceneAndVisualizer();

    while (true)
    {
        if (!m_visualizer->processEvents())
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
                setState(ProgramState::USER_INPUT);
                break;
            case ProgramState::USER_INPUT:
            {
                const auto request = receiveUserMillingRequest();
                if (request.has_value())
                {
                    startSelectedOperation(*request);
                    setState(ProgramState::CUT_USER_SELECTION);
                }
                break;
            }
            case ProgramState::CUT_USER_SELECTION:
                if (m_pathGenerator->isGenerationFinished())
                {
                    finishSelectedOperation();
                    setState(ProgramState::PLAY);
                }
                break;
            case ProgramState::PLAY:
                m_visualizer->startToolPathPlayback();
                setState(ProgramState::FINISHED);
                break;
            default:
                break;
        }
    }

    stopPathGeneration();
}

void InteractiveTorusMillingProgram::initializeSceneAndVisualizer()
{
    m_programContext.emplace(createMillingContext());
    visualization::Visualizer::Config config;
    config.windowTitle = "Transmuter: interactive torus milling";
    m_visualizer = std::make_unique<visualization::Visualizer>(m_programContext->geometryContext,
        m_programContext->pathContext, config);
}

std::optional<visualization::UserMillingRequest> InteractiveTorusMillingProgram::receiveUserMillingRequest()
{
    return m_visualizer->takeMillingRequest();
}

void InteractiveTorusMillingProgram::startSelectedOperation(const visualization::UserMillingRequest& request)
{
    const auto selectedStock = m_programContext->geometryContext.workpiece()->intersection(*request.selectedVolume,
        request.volumeToWorkpiece);
    auto operationScene = m_programContext->geometryContext;
    operationScene.setStock(request.selectedVolume, request.volumeToWorkpiece);
    operationScene.setWorkpiece(selectedStock->clone());
    m_pathGenerator = std::make_unique<pathgenerator::PathGenerator>(request.algorithm, request.config, operationScene,
        *m_programContext->pathContext.machineSetup(), stopSignal());
    m_pathGenerator->startGeneration();
    m_visualizer->setPreviewMachinePath(m_pathGenerator->operationPath());
}

void InteractiveTorusMillingProgram::finishSelectedOperation()
{
    const auto result = m_pathGenerator->getResult();
    *m_programContext->pathContext.machinePath() += *result.machinePath;
    m_programContext->geometryContext.workpiece()->cut(*result.stock, result.stockToWorkpiece);
    m_programContext->geometryContext.workpiece()->combine(*result.workpiece);
    m_visualizer->setPreviewMachinePath({});
}

void InteractiveTorusMillingProgram::stopPathGeneration()
{
    if (m_pathGenerator)
    {
        m_pathGenerator->stopGeneration();
    }
}

MillingProgramContext InteractiveTorusMillingProgram::createMillingContext() const
{
    pathgenerator::MachineConfig machineConfig;
    machineConfig.jointPositionInMachine = Eigen::Vector3d(0.0, 0.0, 100.0);
    machineConfig.workpieceCenterFromJoint = Eigen::Vector3d(0.0, 0.0, 50.0);
    machineConfig.minimumTilt = -std::numbers::pi / 2;
    machineConfig.maximumTilt = std::numbers::pi / 2;
    machineConfig.hasBacklash = {true, true, false, false, false};
    machineConfig.takeupDistance.fill(0.0);

    const auto stock = geometry::GeometryFactory::createDodecahedron(100.0, geometry::GeometryRepresentation::POINT_CLOUD_SOLID, 3.0);
    const auto model = geometry::GeometryFactory::createTorus(50.0, 20.0, geometry::GeometryRepresentation::POINT_CLOUD_SOLID, 3.0);
    const auto tool = geometry::GeometryFactory::createCylinder(10.0, 80.0, geometry::GeometryRepresentation::ANALYTIC);
    auto machine = pathgenerator::createTrunnionFiveAxisMachine(machineConfig, {.sampleSpacing = 5.0});
    return {geometry::MillingScene(stock, model, stock->clone(), tool), pathgenerator::PathGenerationContext(pathgenerator::createMachinePath(), machine)};
}

}  // namespace transmuter::controller::programs
