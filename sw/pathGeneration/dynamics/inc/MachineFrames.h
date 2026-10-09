#pragma once

#include <Eigen/Geometry>

namespace transmuter::pathgenerator
{
struct MachineFrames
{
    Eigen::Affine3d workpiece = Eigen::Affine3d::Identity();
    Eigen::Affine3d tool = Eigen::Affine3d::Identity();
};
}  // namespace transmuter::pathgenerator
