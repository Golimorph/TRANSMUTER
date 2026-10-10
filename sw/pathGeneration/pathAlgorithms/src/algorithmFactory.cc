#include "FiveAxisNormalAlgorithm.h"
#include "PathAlgorithmIf.h"
#include "ThreeAxisRasterAlgorithm.h"
#include "ThreeAxisSpiralAlgorithm.h"
#include "trace.h"
namespace transmuter::pathgenerator::algorithms
{
std::unique_ptr<PathAlgorithmIf> createPathAlgorithm(PathAlgorithmType type, const PathAlgorithmConfig& config, const geometry::MillingScene& scene, MachineKinematicsIf& machine)
{
    switch (type)
    {
        case PathAlgorithmType::THREE_AXIS_RASTER:
            return std::make_unique<ThreeAxisRasterAlgorithm>(scene, machine, config);
        case PathAlgorithmType::THREE_AXIS_SPIRAL:
            return std::make_unique<ThreeAxisSpiralAlgorithm>(scene, machine, config);
        case PathAlgorithmType::FIVE_AXIS_NORMAL:
            return std::make_unique<FiveAxisNormalAlgorithm>(scene, machine, config);
    }
    ERROR("Unknown path algorithm.");
}
}  // namespace transmuter::pathgenerator::algorithms
