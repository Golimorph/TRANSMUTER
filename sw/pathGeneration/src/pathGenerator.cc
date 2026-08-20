#include "pathGenerator.h"

#include <3rdParty/magic_enum.hpp>

#include "pathAlgorithmIf.h"
#include "threeAxisSpiralAlgorithm.h"
#include "trace.h"

namespace transmuter::pathgenerator
{

PathGenerator::PathGenerator(PathGeneratorConfig config,
                             const transmuter::geometry::MillingScene& millingScene)
{
    INFO("Setting algorithm to: " << magic_enum::enum_name(config.algorithmType).data() << ".");
    switch (config.algorithmType)
    {
        case AlgorithmType::THREE_AXIS_SPIRAL:
            m_algorithm =
                std::make_unique<transmuter::pathgenerator::algorithms::ThreeAxisSpiralAlgorithm>(
                    millingScene);
            break;
        default:
            ERROR("Unknown algorithm type selected.");
    }
    m_algorithm->setConfig(config);
}

PathGenerator::~PathGenerator()
{
    stopGeneration();
}

bool PathGenerator::isGenerationInProgress() const
{
    return m_stop == false;
}

bool PathGenerator::isGenerationFinished() const
{
    return m_isToolPathReady.load() == true && m_stop == true;
}

void PathGenerator::startGeneration()
{
    stopGeneration();
    launchGenerationThread();
}

void PathGenerator::stopGeneration()
{
    m_stop = true;
    m_isToolPathReady.store(false);
    INFO("Stop signal sent to algorithm.");
    if (m_thread.joinable())
    {
        m_thread.join();
    }
}

void PathGenerator::launchGenerationThread()
{
    INFO("Launching background path generation thread.");
    m_isToolPathReady.store(false);
    m_stop = false;
    m_thread = std::thread(
        [this]()
        {
            m_algorithm->generate(m_stop);
            m_isToolPathReady.store(true);
            m_stop = true;
            INFO("Path generation finished successfully.");
        });
}

}  // namespace transmuter::pathgenerator