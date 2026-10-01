#ifndef TRANSMUTER_GEOMETRY_POINT_CLOUD_SHAPES_DODECAHEDRON_GENERATOR_H
#define TRANSMUTER_GEOMETRY_POINT_CLOUD_SHAPES_DODECAHEDRON_GENERATOR_H

#include <memory>

#include "pointCloud.h"

namespace transmuter::geometry::pointcloudshapes
{

class DodecahedronGenerator
{
   public:
    DodecahedronGenerator() = delete;

    /** Generate samples on a regular dodecahedron with the requested circumradius. */
    static std::shared_ptr<PointCloud> generate(double circumradius, double density);

    /** Generate samples throughout a regular dodecahedron with the requested circumradius. */
    static std::shared_ptr<PointCloud> generateSolid(double circumradius, double density);
};

}  // namespace transmuter::geometry::pointcloudshapes

#endif  // TRANSMUTER_GEOMETRY_POINT_CLOUD_SHAPES_DODECAHEDRON_GENERATOR_H
