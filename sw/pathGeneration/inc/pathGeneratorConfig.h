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
    double toolDiameter;
    double stepOver;
    double maxDepth;
};

}  // namespace transmuter::pathgenerator
#endif  // TRANSMUTER_PATH_GENERATOR_PATHGENERATORCONFIG_H