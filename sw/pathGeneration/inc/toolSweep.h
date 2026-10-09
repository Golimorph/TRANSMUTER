#pragma once

#include <Eigen/Geometry>
#include <vector>

#include "vertex.h"

namespace transmuter::pathgenerator
{
/** Tool transforms along one proposed workpiece-relative move, including its endpoint. */
std::vector<Eigen::Affine3d> sampleToolSweep(const geometry::Vertex& from, const geometry::Vertex& to, double spacing, double toolReach);
}  // namespace transmuter::pathgenerator
