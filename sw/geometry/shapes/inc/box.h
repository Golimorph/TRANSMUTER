#ifndef TRANSMUTER_GEOMETRY_SHAPES_BOX_H
#define TRANSMUTER_GEOMETRY_SHAPES_BOX_H

#include <Eigen/Dense>
#include <memory>

#include "pointCloud.h"
#include "vertex.h"

namespace transmuter::geometry::shapes
{

/**
 * @brief Generator for Box-style workpieces.
 * * This class provides static methods to generate a point cloud representing
 * a six-sided box. It is specifically designed to test the 5-axis
 * kinematics when encountering sharp 90-degree normal transitions.
 */
class Box
{
   public:
    Box() = delete;

    /**
     * @brief Generates a point-cloud representation of a box.
     * * @param dimensions Vector containing [width, height, depth].
     * @param density The distance between points on the surface (mm).
     * @return A PointCloud object populated with Vertices and Normals.
     */
    static std::shared_ptr<PointCloud> generate(const Eigen::Vector3f& dimensions, float density);
};

}  // namespace transmuter::geometry::shapes

#endif