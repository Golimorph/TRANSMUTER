#pragma once

#include <Eigen/Geometry>

#include "vertex.h"

namespace transmuter::pathgenerator
{
struct MachineDisplayPose
{
    geometry::Vertex toolInWorkpiece;
    Eigen::Affine3d workpiece;
    Eigen::Affine3d tool;
};
}  // namespace transmuter::pathgenerator
