#ifndef TRANSMUTER_PATH_GENERATOR_PATHGENERATORCONFIG_H
#define TRANSMUTER_PATH_GENERATOR_PATHGENERATORCONFIG_H

namespace transmuter::pathgenerator
{

enum class AlgorithmType
{
    THREE_AXIS_SPIRAL
};

enum class Direction
{
    FORWARD,
    REVERSE
};

struct PathGeneratorConfig
{
    AlgorithmType algorithmType;
    Direction direction;
    double pointSpacing;  // Maximum distance between consecutive tool-path samples.
    double passSpacing;   // Distance between neighbouring passes on the same layer.
    double layerSpacing;  // Distance between neighbouring depth layers.
};

}  // namespace transmuter::pathgenerator
#endif  // TRANSMUTER_PATH_GENERATOR_PATHGENERATORCONFIG_H
