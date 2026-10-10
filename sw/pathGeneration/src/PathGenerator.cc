#include "PathGenerator.h"

#include <3rdParty/magic_enum.hpp>

#include "MachineKinematicsIf.h"
#include "PathAlgorithmIf.h"
#include "machineFactory.h"
#include "trace.h"

namespace transmuter::pathgenerator
{

PathGenerator::PathGenerator(PathAlgorithmType algorithmType, PathAlgorithmConfig config, const geometry::MillingScene& millingScene, const MachineKinematicsIf& machineSetup, std::atomic<bool>& stopRequested)
    : m_algorithmType(algorithmType), m_config(config), m_millingScene(millingScene), m_machineSetup(machineSetup), m_stopRequested(stopRequested)
{
    INFO("Setting algorithm to: " << magic_enum::enum_name(algorithmType).data() << ".");
}

PathGenerator::~PathGenerator()
{
    stopGeneration();
}

bool PathGenerator::isGenerationFinished() const
{
    return m_isResultReady.load();
}

PathGenerationResult PathGenerator::getResult() const
{
    if (!isGenerationFinished())
    {
        ERROR("Path generation result requested before generation finished.");
    }
    return m_result;
}

std::shared_ptr<const MachinePathIf> PathGenerator::operationPath() const
{
    if (!m_machineKinematics)
    {
        ERROR("Operation path requested before path generation started.");
    }
    return m_machineKinematics->path();
}

void PathGenerator::startGeneration()
{
    if (m_stopRequested.load() || (m_thread.joinable() && !isGenerationFinished()))
    {
        ERROR("Cannot start path generation while running or after stop was requested.");
    }
    stopGeneration();
    launchGenerationThread();
}

void PathGenerator::stopGeneration()
{
    const bool wasGenerating = m_thread.joinable() && !isGenerationFinished();
    if (wasGenerating)
    {
        m_stopRequested = true;
    }
    if (m_thread.joinable())
    {
        if (wasGenerating)
        {
            INFO("Cancelling active path generation.");
        }
        m_thread.join();
    }
    m_algorithm.reset();
    m_machineKinematics.reset();
    m_isResultReady.store(false);
}

void PathGenerator::launchGenerationThread()
{
    INFO("Launching background path generation thread.");
    m_result = {};
    m_isResultReady.store(false);
    const auto emptyPath = createMachinePath();
    m_machineKinematics = m_machineSetup.fork(*emptyPath);
    m_algorithm = algorithms::createPathAlgorithm(m_algorithmType, m_config, m_millingScene, *m_machineKinematics);
    m_thread = std::thread(&PathGenerator::generatePath, this);
}

void PathGenerator::generatePath()
{
    m_result = m_algorithm->generate(m_stopRequested);
    m_isResultReady.store(!m_stopRequested.load());
    INFO("Path generation finished.");
}

}  // namespace transmuter::pathgenerator
