#include "threeAxisRasterAlgorithm.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "threeAxisRasterCuttingOperation.h"
#include "trace.h"

namespace transmuter::pathgenerator::algorithms
{

ThreeAxisRasterAlgorithm::ThreeAxisRasterAlgorithm(
    const transmuter::geometry::MillingScene& scene, PathGeneratorConfig config)
    : PathAlgorithmIf(scene), m_config(std::move(config))
{
    validateConfig(m_config);
    INFO("Raster algorithm configured: point spacing "
         << m_config.pointSpacing << " mm, pass spacing " << m_config.passSpacing
         << " mm, layer spacing " << m_config.layerSpacing << " mm, geometry tolerance "
         << m_config.geometryTolerance << " mm.");
}

void ThreeAxisRasterAlgorithm::validateConfig(const PathGeneratorConfig& config)
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

PathGenerationResult ThreeAxisRasterAlgorithm::generate(std::atomic<bool>& stopRequested, std::shared_ptr<geometry::ToolPathIf> pathOutput) const
{
    if (!pathOutput)
    {
        ERROR("tool path output is null");
    }

    ThreeAxisRasterCuttingOperation cuttingOperation(stock(), model(), toolModel(), m_config.geometryTolerance, *pathOutput);

    const auto layers = cuttingOperation.layers(m_config.layerSpacing);
    const auto rows = cuttingOperation.rows(m_config.passSpacing);
    INFO("Starting unidirectional raster generation with " << layers.size() << " layers and " << rows.size() << " rows per layer.");

    for (const auto& layer : layers)
    {
        if (stopRequested.load())
        {
            INFO("Stop signal received before starting the next layer.");
            break;
        }

        INFO("Generating raster layer at " << layer.toolZ << " mm.");
        cuttingOperation.prepareToCut(layer);
        for (const auto& row : rows)
        {
            cuttingOperation.beginRow(row);
            cutRowFromLowToHighXAvoidingModel(cuttingOperation);
            cuttingOperation.finishCurrentRow();
        }
        cuttingOperation.finishCurrentLayer();
    }

    INFO("Generated " << cuttingOperation.completedLayerCount() << " unidirectional raster layers.");
    return PathGenerationResult{
        .toolPath = std::move(pathOutput),
        .workpiece = cuttingOperation.takeWorkpiece(),
    };
}

void ThreeAxisRasterAlgorithm::cutRowFromLowToHighXAvoidingModel(ThreeAxisRasterCuttingOperation& cuttingOperation) const
{
    const double rowLength = cuttingOperation.rowEndX() - cuttingOperation.rowStartX();
    const size_t sampleCount = std::max<size_t>(1, static_cast<size_t>(std::ceil(rowLength / m_config.pointSpacing)));

    for (size_t sample = 0; sample <= sampleCount; ++sample)
    {
        const double progress = static_cast<double>(sample) / sampleCount;
        const double toolX = cuttingOperation.rowStartX() + progress * rowLength;
        cuttingOperation.moveToolAlongRowAvoidingModel(toolX);
    }
}

}  // namespace transmuter::pathgenerator::algorithms
