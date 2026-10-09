#pragma once

#include <cstddef>

namespace transmuter::pathgenerator
{
/** Geometric strategy settings; no machine or validation policy. */
struct PathAlgorithmConfig
{
    // Maximum distance between requested cutting points, neighboring passes, and depth layers.
    double pointSpacing = 0.0;
    double passSpacing = 0.0;
    double layerSpacing = 0.0;
    double geometryTolerance = 0.0;
    // Exposed-surface resolution, independent of machine checking.
    double surfaceSampleSpacing = 3.0;
    size_t maximumSurfaceLayers = 100;
};
}  // namespace transmuter::pathgenerator
