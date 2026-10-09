#include "PathGenerator.h"

#include <3rdParty/magic_enum.hpp>
#include <vector>

#include "MachineKinematicsIf.h"
#include "PathAlgorithmIf.h"
#include "machineFactory.h"
#include "trace.h"

namespace transmuter::pathgenerator
{

PathGenerator::PathGenerator(PathAlgorithmType algorithmType, PathAlgorithmConfig config, const geometry::MillingScene& millingScene, std::atomic<bool>& stopRequested)
    : m_machineKinematics(millingScene.machineKinematics), m_stopRequested(stopRequested)
{
    if (!m_machineKinematics)
    {
        ERROR("Path generation requires machine kinematics.");
    }

    INFO("Setting algorithm to: " << magic_enum::enum_name(algorithmType).data() << ".");
    m_algorithm = algorithms::createPathAlgorithm(algorithmType, config, millingScene);
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
    m_isResultReady.store(false);
}

void PathGenerator::launchGenerationThread()
{
    INFO("Launching background path generation thread.");
    m_result = {};
    m_isResultReady.store(false);
    m_thread = std::thread(&PathGenerator::generatePath, this);
}

void PathGenerator::generatePath()
{
    m_machineKinematics->clearPath();
    m_result = m_algorithm->generate(m_stopRequested);
    auto completedPath = createMachinePath();
    std::vector<MachineMove> moves;
    m_result.machinePath->copyTo(moves);
    for (const auto& move : moves)
    {
        completedPath->append(move);
    }
    m_result.machinePath = completedPath;
    m_isResultReady.store(!m_stopRequested.load());
    INFO("Path generation finished.");
}

}  // namespace transmuter::pathgenerator
