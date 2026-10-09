#pragma once

#include <Eigen/Core>
#include <array>
#include <numbers>

namespace transmuter::pathgenerator
{
/** Physical setup shared by the fixed-table and moving-trunnion machine models. */
struct MachineConfig
{
    Eigen::Vector3d jointPositionInMachine{0.0, 0.0, 100.0};
    Eigen::Vector3d workpieceCenterFromJoint{0.0, 0.0, 50.0};
    double minimumTilt = -std::numbers::pi;
    double maximumTilt = std::numbers::pi;
    std::array<bool, 5> hasBacklash{true, true, false, false, false};  // X, Y, Z, A, B.
    std::array<double, 5> takeupDistance{};  // Zero until measured; ideal instantaneous take-up.
};
}  // namespace transmuter::pathgenerator
