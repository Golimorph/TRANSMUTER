#ifndef TRANSMUTER_GEOMETRY_POINT_CLOUD_SHAPES_BOX_GENERATOR_H
#define TRANSMUTER_GEOMETRY_POINT_CLOUD_SHAPES_BOX_GENERATOR_H

#include <Eigen/Dense>
#include <memory>

#include "../../../pointCloud/inc/pointCloud.h"

namespace transmuter::geometry::pointcloudshapes
{

/**
 * @brief Generates point-cloud boxes for visualization and solid operations.
 */
class BoxGenerator
{
   public:
    BoxGenerator() = delete;

    /**
     * @brief Generates a point-cloud representation of a box.
     * @param dimensions Vector containing [width, height, depth].
     * @param density The distance between points on the surface (mm).
     * @return A PointCloud object populated with Vertices and Normals.
     */
    static std::shared_ptr<PointCloud> generate(const Eigen::Vector3d& dimensions, double density);

    /** Generate samples throughout the box volume for solid operations. */
    static std::shared_ptr<PointCloud> generateSolid(const Eigen::Vector3d& dimensions, double density);
};

}  // namespace transmuter::geometry::pointcloudshapes

#endif  // TRANSMUTER_GEOMETRY_POINT_CLOUD_SHAPES_BOX_GENERATOR_H
