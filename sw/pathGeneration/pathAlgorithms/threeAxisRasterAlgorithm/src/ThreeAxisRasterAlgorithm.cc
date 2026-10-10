#include "ThreeAxisRasterAlgorithm.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "ThreeAxisRasterCuttingOperation.h"
#include "trace.h"

namespace transmuter::pathgenerator::algorithms
{

ThreeAxisRasterAlgorithm::ThreeAxisRasterAlgorithm(const transmuter::geometry::MillingScene& scene, MachineKinematicsIf& machine, PathAlgorithmConfig config)
    : PathAlgorithmIf(scene, machine), m_config(std::move(config))
{
    validateConfig(m_config);
    INFO("Raster algorithm configured: point spacing "
         << m_config.pointSpacing << " mm, pass spacing " << m_config.passSpacing << " mm, layer spacing "
         << m_config.layerSpacing << " mm, geometry tolerance " << m_config.geometryTolerance << " mm.");
}

void ThreeAxisRasterAlgorithm::validateConfig(const PathAlgorithmConfig& config)
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

PathGenerationResult ThreeAxisRasterAlgorithm::generate(const std::atomic<bool>& stopRequested)
{
    ThreeAxisRasterCuttingOperation cuttingOperation(operationStock(), model(), toolModel(),
        m_config.geometryTolerance, workpiece(), machine(), stopRequested);

    cutLayers(cuttingOperation, stopRequested);
    return PathGenerationResult{
        .machinePath = machine().path(),
        .stock = scene().stock(),
        .stockToWorkpiece = scene().stockToWorkpiece(),
        .workpiece = finishedWorkpiece(),
    };
}

void ThreeAxisRasterAlgorithm::cutLayers(ThreeAxisRasterCuttingOperation& cuttingOperation, const std::atomic<bool>& stopRequested) const
{
    const auto layers = cuttingOperation.layers(m_config.layerSpacing);
    INFO("Starting raster generation with " << layers.size() << " layers; cutting each row in +X.");
    for (const auto& layer : layers)
    {
        if (stopRequested.load())
        {
            break;
        }
        INFO("Generating raster layer at " << layer.toolZ << " mm.");
        cuttingOperation.prepareToCut(layer);
        cutRows(cuttingOperation, stopRequested);
        if (!stopRequested.load())
        {
            cuttingOperation.finishCurrentLayer();
        }
    }
    INFO("Generated " << cuttingOperation.completedLayerCount() << " complete raster layers.");
}
void ThreeAxisRasterAlgorithm::cutRows(ThreeAxisRasterCuttingOperation& cuttingOperation, const std::atomic<bool>& stopRequested) const
{
    auto rows = cuttingOperation.rows(m_config.passSpacing);
    bool firstRowInLayer = true;
    for (const auto& row : rows)
    {
        if (stopRequested.load())
        {
            break;
        }
        cuttingOperation.beginRow(row, firstRowInLayer);
        firstRowInLayer = false;
        cutRowFromLowToHighXAvoidingModel(cuttingOperation, stopRequested);
    }
}

void ThreeAxisRasterAlgorithm::cutRowFromLowToHighXAvoidingModel(ThreeAxisRasterCuttingOperation& cuttingOperation, const std::atomic<bool>& stopRequested) const
{
    const double rowLength = cuttingOperation.rowEndX() - cuttingOperation.rowStartX();
    const size_t sampleCount = std::max<size_t>(1,
        static_cast<size_t>(std::ceil(rowLength / m_config.pointSpacing)));

    for (size_t sample = 0; sample <= sampleCount && !stopRequested.load(); ++sample)
    {
        const double progress = static_cast<double>(sample) / sampleCount;
        const double toolX = cuttingOperation.rowStartX() + progress * rowLength;
        cuttingOperation.moveToolAlongRowAvoidingModel(toolX);
    }
}

}  // namespace transmuter::pathgenerator::algorithms
