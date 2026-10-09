#ifndef TRANSMUTER_PATHGEN_THREE_AXIS_RASTER_ALGORITHM_H
#define TRANSMUTER_PATHGEN_THREE_AXIS_RASTER_ALGORITHM_H

#include <atomic>
#include <memory>

#include "PathAlgorithmIf.h"

namespace transmuter::pathgenerator::algorithms
{

class ThreeAxisRasterCuttingOperation;

class ThreeAxisRasterAlgorithm : public PathAlgorithmIf
{
   public:
    ThreeAxisRasterAlgorithm(const transmuter::geometry::MillingScene& scene, PathAlgorithmConfig config);
    ~ThreeAxisRasterAlgorithm() override = default;

    PathGenerationResult generate(const std::atomic<bool>& stopRequested) override;

   private:
    void cutLayers(ThreeAxisRasterCuttingOperation& operation, const std::atomic<bool>& stop) const;
    void cutRows(ThreeAxisRasterCuttingOperation& operation, const std::atomic<bool>& stop) const;
    static void validateConfig(const PathAlgorithmConfig& config);
    void cutRowFromLowToHighXAvoidingModel(ThreeAxisRasterCuttingOperation& cuttingOperation, const std::atomic<bool>& stopRequested) const;

    const PathAlgorithmConfig m_config;
};

}  // namespace transmuter::pathgenerator::algorithms

#endif  // TRANSMUTER_PATHGEN_THREE_AXIS_RASTER_ALGORITHM_H
