#pragma once
#include "geometryIf.h"
namespace transmuter::pathgenerator::algorithms
{
class ExposedSurface
{
   public:
    static std::vector<geometry::Vertex> sample(const geometry::GeometryIf& workpiece, double materialSpacing, double passSpacing);
};
}  // namespace transmuter::pathgenerator::algorithms
