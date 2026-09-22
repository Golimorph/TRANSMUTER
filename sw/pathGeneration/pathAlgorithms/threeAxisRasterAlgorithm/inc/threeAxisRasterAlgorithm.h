#ifndef TRANSMUTER_PATHGEN_THREE_AXIS_RASTER_ALGORITHM_H
#define TRANSMUTER_PATHGEN_THREE_AXIS_RASTER_ALGORITHM_H

#include <atomic>
#include <memory>

#include "pathAlgorithmIf.h"

namespace transmuter::pathgenerator::algorithms
{

class ThreeAxisRasterCuttingOperation;

class ThreeAxisRasterAlgorithm : public PathAlgorithmIf
{
   public:
    ThreeAxisRasterAlgorithm(
        const transmuter::geometry::MillingScene& scene, PathGeneratorConfig config);
    ~ThreeAxisRasterAlgorithm() override = default;

    PathGenerationResult generate(
        std::atomic<bool>& stopRequested,
        std::shared_ptr<geometry::ToolPathIf> pathOutput) const override;

   private:
    static void validateConfig(const PathGeneratorConfig& config);
    void cutRowFromLowToHighXAvoidingModel(
        ThreeAxisRasterCuttingOperation& cuttingOperation) const;

    const PathGeneratorConfig m_config;
};

}  // namespace transmuter::pathgenerator::algorithms

#endif  // TRANSMUTER_PATHGEN_THREE_AXIS_RASTER_ALGORITHM_H
