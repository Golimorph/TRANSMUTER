#include "threeAxisSpiralAlgorithm.h"

#include <utility>

#include "threeAxisSpiralCuttingOperation.h"
#include "trace.h"

namespace transmuter::pathgenerator::algorithms
{

ThreeAxisSpiralAlgorithm::ThreeAxisSpiralAlgorithm(
    const transmuter::geometry::MillingScene& scene)
    : PathAlgorithmIf(scene)
{
}

void ThreeAxisSpiralAlgorithm::setConfig(const PathGeneratorConfig& config)
{
    if (config.pointSpacing <= 0.0 || config.passSpacing <= 0.0 ||
        config.layerSpacing <= 0.0)
    {
        ERROR("Path, pass, and layer spacing must be greater than zero.");
    }
    m_config = config;
    INFO("SpiralAlgo: Configuration updated.");
}

PathGenerationResult ThreeAxisSpiralAlgorithm::generate(
    std::atomic<bool>& stopRequested,
    std::shared_ptr<geometry::PointCloud> liveToolPath)
{
    ThreeAxisSpiralCuttingOperation cuttingOperation(stock(), model(), toolModel(), std::move(liveToolPath));

    const auto layers = cuttingOperation.layersFromTopToBottom(m_config.layerSpacing);
    const auto circles = cuttingOperation.concentricCircles(m_config.passSpacing);
    for (const auto& layer : layers)
    {
        if (stopRequested.load())
        {
            break;
        }

        cuttingOperation.prepareToCut(layer);
        for (const auto& circle : circles)
        {
            cuttingOperation.cutCounterClockwise(circle, m_config.pointSpacing);
        }

        cuttingOperation.finishCurrentLayerAtCenter();
    }

    INFO("SpiralAlgo: Generated " << cuttingOperation.completedLayerCount()
                                  << " counter-clockwise layers.");
    return cuttingOperation.takeResult();
}

}  // namespace transmuter::pathgenerator::algorithms
