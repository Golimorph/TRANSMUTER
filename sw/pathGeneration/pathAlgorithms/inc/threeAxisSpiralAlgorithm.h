#ifndef TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_ALGORITHM_H
#define TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_ALGORITHM_H

#include "pathAlgorithmIf.h"

namespace transmuter::pathgenerator::algorithms
{

class ThreeAxisSpiralAlgorithm : public PathAlgorithmIf
{
   public:
    ThreeAxisSpiralAlgorithm(const transmuter::geometry::MillingScene& scene);
    virtual ~ThreeAxisSpiralAlgorithm() = default;

    void setConfig(const PathGeneratorConfig& config) override;
    void generate(std::atomic<bool>& stopSignal) override;

   private:
    PathGeneratorConfig m_config;
};

}  // namespace transmuter::pathgenerator::algorithms

#endif  // TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_ALGORITHM_H