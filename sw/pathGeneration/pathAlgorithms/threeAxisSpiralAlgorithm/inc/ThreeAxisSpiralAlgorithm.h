#ifndef TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_ALGORITHM_H
#define TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_ALGORITHM_H

#include <atomic>
#include <memory>

#include "PathAlgorithmIf.h"

namespace transmuter::pathgenerator::algorithms
{

class ThreeAxisSpiralCuttingOperation;

class ThreeAxisSpiralAlgorithm : public PathAlgorithmIf
{
   public:
    ThreeAxisSpiralAlgorithm(const transmuter::geometry::MillingScene& scene, PathAlgorithmConfig config);
    ~ThreeAxisSpiralAlgorithm() override = default;

    PathGenerationResult generate(const std::atomic<bool>& stopRequested) override;

   private:
    void cutLayers(ThreeAxisSpiralCuttingOperation& operation, const std::atomic<bool>& stop) const;
    static void validateConfig(const PathAlgorithmConfig& config);
    void cutCounterClockwiseAvoidingModel(ThreeAxisSpiralCuttingOperation& cuttingOperation, double circleRadius, const std::atomic<bool>& stopRequested) const;

    const PathAlgorithmConfig m_config;
};

}  // namespace transmuter::pathgenerator::algorithms

#endif  // TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_ALGORITHM_H
