#include "FiveAxisNormalAlgorithm.h"

#include <cmath>
#include <utility>

#include "NormalSurfaceCuttingOperation.h"
#include "trace.h"

namespace transmuter::pathgenerator::algorithms
{
FiveAxisNormalAlgorithm::FiveAxisNormalAlgorithm(const geometry::MillingScene& scene, MachineKinematicsIf& machine, PathAlgorithmConfig config)
    : PathAlgorithmIf(scene, machine), m_config(std::move(config))
{
    for (double spacing : {m_config.pointSpacing, m_config.passSpacing, m_config.layerSpacing,
             m_config.geometryTolerance, m_config.surfaceSampleSpacing})
        if (!std::isfinite(spacing) || spacing <= 0.0)
        {
            ERROR("Normal-surface spacings must be positive and finite.");
        }
    if (m_config.maximumSurfaceLayers == 0)
    {
        ERROR("At least one surface layer must be requested.");
    }
}
PathGenerationResult FiveAxisNormalAlgorithm::generate(const std::atomic<bool>& stop)
{
    NormalSurfaceCuttingOperation operation(scene(), m_config, workpiece(), machine(), stop);
    for (size_t layer = 0; layer < m_config.maximumSurfaceLayers && !stop.load(); ++layer)
    {
        INFO("Peeling sampled stock surface " << layer + 1);
        if (!operation.peelExposedSurface())
        {
            break;
        }
    }
    return {
        .machinePath = machine().path(),
        .stock = scene().stock(),
        .stockToWorkpiece = scene().stockToWorkpiece(),
        .workpiece = finishedWorkpiece(),
        .generationNote = operation.generationNote(stop.load()),
    };
}
}  // namespace transmuter::pathgenerator::algorithms
