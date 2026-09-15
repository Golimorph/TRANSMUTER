#ifndef TRANSMUTER_GEOMETRY_POINT_CLOUD_SHAPES_TORUS_GENERATOR_H
#define TRANSMUTER_GEOMETRY_POINT_CLOUD_SHAPES_TORUS_GENERATOR_H

#include <memory>

#include "../../../pointCloud/inc/pointCloud.h"

namespace transmuter::geometry::pointcloudshapes
{

class TorusGenerator
{
   public:
    TorusGenerator() = delete;

    /** Generate samples on a ring torus centered on the origin and aligned with +Z. */
    static std::shared_ptr<PointCloud> generate(double majorRadius, double tubeRadius, double density);

    /** Generate samples throughout a ring torus centered on the origin and aligned with +Z. */
    static std::shared_ptr<PointCloud> generateSolid(double majorRadius, double tubeRadius, double density);
};

}  // namespace transmuter::geometry::pointcloudshapes

#endif  // TRANSMUTER_GEOMETRY_POINT_CLOUD_SHAPES_TORUS_GENERATOR_H
