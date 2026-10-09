#pragma once
#include "geometryIf.h"
namespace transmuter::pathgenerator::algorithms
{
/** Contact sampling only. GeometryIf continues to own all material removal. */
class MaterialEngagement
{
   public:
    static std::vector<Eigen::Vector3d> sample(const geometry::GeometryIf& stock, const geometry::GeometryIf& tool, const Eigen::Affine3d& pose, double spacing);
};
}  // namespace transmuter::pathgenerator::algorithms
