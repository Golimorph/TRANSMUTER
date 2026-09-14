#ifndef TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_ALGORITHM_H
#define TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_ALGORITHM_H

#include <atomic>
#include <memory>

#include "pathAlgorithmIf.h"
#include "pointCloud.h"

namespace transmuter::pathgenerator::algorithms
{

class ThreeAxisSpiralAlgorithm : public PathAlgorithmIf
{
   public:
    explicit ThreeAxisSpiralAlgorithm(const transmuter::geometry::MillingScene& scene);
    ~ThreeAxisSpiralAlgorithm() override = default;

    void setConfig(const PathGeneratorConfig& config) override;
    PathGenerationResult generate(std::atomic<bool>& stopRequested, std::shared_ptr<geometry::PointCloud> liveToolPath) override;

   private:
    PathGeneratorConfig m_config;
};

}  // namespace transmuter::pathgenerator::algorithms

#endif  // TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_ALGORITHM_H
