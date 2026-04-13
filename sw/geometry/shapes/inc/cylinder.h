#ifndef TRANSMUTER_GEOMETRY_SHAPES_CYLINDER_H
#define TRANSMUTER_GEOMETRY_SHAPES_CYLINDER_H

#include <Eigen/Dense>
#include <memory>

#include "pointCloud.h"
#include "vertex.h"

namespace transmuter::geometry::shapes
{

/**
 * @brief Generator for Cylinder-style workpieces.
 * This class provides static methods to generate a point cloud representing
 * a cylinder. It is designed for testing milling algorithms on curved surfaces.
 */
class Cylinder
{
   public:
    Cylinder() = delete;

    /**
     * @brief Generates a point-cloud representation of a cylinder.
     * @param dimensions Vector containing [radius, height, unused].
     * @param density The distance between points on the surface (mm).
     * @return A PointCloud object populated with Vertices and Normals.
     */
    static std::shared_ptr<PointCloud> generate(const float radius, const float height,
                                                const float density);
};

}  // namespace transmuter::geometry::shapes

#endif