#ifndef TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_ALGORITHM_H
#define TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_ALGORITHM_H

#include <atomic>
#include <memory>

#include "pathAlgorithmIf.h"

namespace transmuter::pathgenerator::algorithms
{

class ThreeAxisSpiralCuttingOperation;

class ThreeAxisSpiralAlgorithm : public PathAlgorithmIf
{
   public:
    ThreeAxisSpiralAlgorithm(
        const transmuter::geometry::MillingScene& scene, PathGeneratorConfig config);
    ~ThreeAxisSpiralAlgorithm() override = default;

    PathGenerationResult generate(
        std::atomic<bool>& stopRequested,
        std::shared_ptr<geometry::ToolPathIf> pathOutput) const override;

   private:
    static void validateConfig(const PathGeneratorConfig& config);
    void cutCounterClockwiseAvoidingModel(
        ThreeAxisSpiralCuttingOperation& cuttingOperation, double circleRadius) const;

    const PathGeneratorConfig m_config;
};

}  // namespace transmuter::pathgenerator::algorithms

#endif  // TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_ALGORITHM_H
