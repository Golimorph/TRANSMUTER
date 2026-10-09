#pragma once

namespace transmuter::pathgenerator
{
/** Sampling policy for checking joint-interpolated machine motion. */
struct MachineMotionConfig
{
    double sampleSpacing = 1.0;
    double pathTolerance = 0.25;
};
}  // namespace transmuter::pathgenerator
