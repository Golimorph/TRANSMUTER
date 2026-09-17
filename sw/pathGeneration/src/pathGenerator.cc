#include "pathGenerator.h"

#include <3rdParty/magic_enum.hpp>

#include "pathAlgorithmIf.h"
#include "threeAxisSpiralAlgorithm.h"
#include "trace.h"

namespace transmuter::pathgenerator
{

namespace
{

std::unique_ptr<algorithms::PathAlgorithmIf> createAlgorithm(PathGenerator::AlgorithmType algorithmType,
    const PathGeneratorConfig& config, const geometry::MillingScene& millingScene)
{
    switch (algorithmType)
    {
        case PathGenerator::AlgorithmType::THREE_AXIS_SPIRAL:
            return std::make_unique<algorithms::ThreeAxisSpiralAlgorithm>(millingScene, config);
        default:
            ERROR("Unknown algorithm type selected.");
    }
    return nullptr;
}

}  // namespace

// Keep the scene-owned path so generation and visualization observe the same object.
PathGenerator::PathGenerator(AlgorithmType algorithmType, PathGeneratorConfig config, const transmuter::geometry::MillingScene& millingScene)
    : m_pathOutput(millingScene.toolPath)
{
    if (!m_pathOutput)
    {
        ERROR("toolPath is null ptr");
    }

    INFO("Setting algorithm to: " << magic_enum::enum_name(algorithmType).data() << ".");
    m_algorithm = createAlgorithm(algorithmType, config, millingScene);
}

PathGenerator::~PathGenerator()
{
    stopGeneration();
}

bool PathGenerator::isGenerationInProgress() const
{
    return m_isGenerating.load();
}

bool PathGenerator::isGenerationFinished() const
{
    return m_isResultReady.load() && !m_isGenerating.load();
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
    stopGeneration();
    launchGenerationThread();
}

void PathGenerator::stopGeneration()
{
    m_stopRequested = true;
    if (m_thread.joinable())
    {
        INFO("Stop signal sent to algorithm.");
        m_thread.join();
    }
    m_isGenerating = false;
    m_isResultReady.store(false);
}

void PathGenerator::launchGenerationThread()
{
    INFO("Launching background path generation thread.");
    m_result = {};
    m_isResultReady.store(false);
    m_stopRequested = false;
    m_isGenerating = true;
    m_thread = std::thread(
        [this]()
        {
            m_result = m_algorithm->generate(m_stopRequested, m_pathOutput);
            const bool wasStopped = m_stopRequested.load();
            m_isResultReady.store(!wasStopped);
            m_isGenerating = false;
            if (wasStopped)
            {
                INFO("Path generation stopped after completing its current layer.");
            }
            else
            {
                INFO("Path generation finished successfully.");
            }
        });
}

}  // namespace transmuter::pathgenerator
