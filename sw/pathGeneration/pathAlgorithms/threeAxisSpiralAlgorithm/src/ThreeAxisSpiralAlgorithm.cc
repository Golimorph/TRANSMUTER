#include "ThreeAxisSpiralAlgorithm.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

#include "ThreeAxisSpiralCuttingOperation.h"
#include "trace.h"

namespace transmuter::pathgenerator::algorithms
{

ThreeAxisSpiralAlgorithm::ThreeAxisSpiralAlgorithm(const transmuter::geometry::MillingScene& scene, PathAlgorithmConfig config)
    : PathAlgorithmIf(scene), m_config(std::move(config))
{
    validateConfig(m_config);
    INFO("Spiral algorithm configured: point spacing "
         << m_config.pointSpacing << " mm, pass spacing " << m_config.passSpacing << " mm, layer spacing "
         << m_config.layerSpacing << " mm, geometry tolerance " << m_config.geometryTolerance << " mm.");
}

void ThreeAxisSpiralAlgorithm::validateConfig(const PathAlgorithmConfig& config)
{
    if (config.pointSpacing <= 0.0)
    {
        ERROR("Point spacing must be greater than 0 mm.");
    }
    if (config.passSpacing <= 0.0)
    {
        ERROR("Pass spacing must be greater than 0 mm.");
    }
    if (config.layerSpacing <= 0.0)
    {
        ERROR("Layer spacing must be greater than 0 mm.");
    }
    if (config.geometryTolerance <= 0.0)
    {
        ERROR("Geometry tolerance must be greater than 0 mm.");
    }
}

PathGenerationResult ThreeAxisSpiralAlgorithm::generate(const std::atomic<bool>& stopRequested)
{
    ThreeAxisSpiralCuttingOperation cuttingOperation(stock(), model(), toolModel(), m_config.geometryTolerance, workpiece(), machine(), stopRequested);

    cutLayers(cuttingOperation, stopRequested);
    return PathGenerationResult{
        .machinePath = machine().path(),
        .workpiece = finishedWorkpiece(),
    };
}

void ThreeAxisSpiralAlgorithm::cutLayers(ThreeAxisSpiralCuttingOperation& cuttingOperation, const std::atomic<bool>& stopRequested) const
{
    const auto layers = cuttingOperation.layers(m_config.layerSpacing);
    const auto circles = cuttingOperation.concentricCircles(m_config.passSpacing);
    INFO("Starting spiral generation with " << layers.size() << " layers and " << circles.size() << " concentric circles per layer.");
    for (const auto& layer : layers)
    {
        if (stopRequested.load())
        {
            INFO("Stop signal received before starting the next layer.");
            break;
        }

        INFO("Generating tool-path layer at " << layer.toolZ << " mm.");
        cuttingOperation.prepareToCut(layer);
        for (const auto& circle : circles)
        {
            if (stopRequested.load())
            {
                break;
            }
            cutCounterClockwiseAvoidingModel(cuttingOperation, circle.radius, stopRequested);
        }

        if (!stopRequested.load())
        {
            cuttingOperation.finishCurrentLayerAtCenter();
        }
    }

    INFO("Generated " << cuttingOperation.completedLayerCount() << " counter-clockwise layers.");
}

void ThreeAxisSpiralAlgorithm::cutCounterClockwiseAvoidingModel(ThreeAxisSpiralCuttingOperation& cuttingOperation, double circleRadius, const std::atomic<bool>& stopRequested) const
{
    const double circumference = 2.0 * std::numbers::pi * circleRadius;
    const size_t sampleCount = std::max<size_t>(3,
        static_cast<size_t>(std::ceil(circumference / m_config.pointSpacing)));

    for (size_t sample = 0; sample <= sampleCount && !stopRequested.load(); ++sample)
    {
        const double angle = 2.0 * std::numbers::pi * sample / sampleCount;
        const Eigen::Vector2d toolCenter = cuttingOperation.center() + Eigen::Vector2d(std::cos(angle), std::sin(angle)) * circleRadius;
        cuttingOperation.moveToolToLayerHeightAvoidingModel(toolCenter);
    }
}

}  // namespace transmuter::pathgenerator::algorithms
