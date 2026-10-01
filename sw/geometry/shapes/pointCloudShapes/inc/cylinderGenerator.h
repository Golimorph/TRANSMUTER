#ifndef TRANSMUTER_GEOMETRY_POINT_CLOUD_SHAPES_CYLINDER_GENERATOR_H
#define TRANSMUTER_GEOMETRY_POINT_CLOUD_SHAPES_CYLINDER_GENERATOR_H

#include <memory>

#include "pointCloud.h"

namespace transmuter::geometry::pointcloudshapes
{

class CylinderGenerator
{
   public:
    CylinderGenerator() = delete;

    /** Generate samples on a cylinder boundary centered on the origin and aligned with +Z. */
    static std::shared_ptr<PointCloud> generate(double radius, double height, double density);

    /** Generate samples throughout a cylinder centered on the origin and aligned with +Z. */
    static std::shared_ptr<PointCloud> generateSolid(double radius, double height, double density);
};

}  // namespace transmuter::geometry::pointcloudshapes

#endif  // TRANSMUTER_GEOMETRY_POINT_CLOUD_SHAPES_CYLINDER_GENERATOR_H
