#pragma once
#include "viewCoordinates.h"

namespace transmuter::visualization
{
struct VisualizerConfig
{
    double toolPathPlaybackDurationSeconds = 60.0;
    // Relative to CUTTING while preserving the configured total playback duration.
    double rapidPlaybackSpeedMultiplier = 5.0;
    ViewCoordinates initialCoordinates = ViewCoordinates::WORKPIECE;
};
}  // namespace transmuter::visualization
