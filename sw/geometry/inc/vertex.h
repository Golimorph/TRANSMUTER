#ifndef TRANSMUTER_GEOMETRY_SHAPES_VERTEX_H
#define TRANSMUTER_GEOMETRY_SHAPES_VERTEX_H

#include <Eigen/Dense>

namespace transmuter::geometry
{

/**
 * A sampled position and normal used for geometry presentation and tool poses.
 */
struct Vertex
{
    Eigen::Vector3f pos;     // [x, y, z]
    Eigen::Vector3f normal;  // [nx, ny, nz]
};

}  // namespace transmuter::geometry
#endif
