#ifndef TRANSMUTER_VERTEX_H
#define TRANSMUTER_VERTEX_H

#include <Eigen/Dense>

namespace transmuter::geometry
{

/* ! @brief a representation of a point in 3D space. This is the fundamental
 * unit for representing an object
 */
struct Vertex
{
    Eigen::Vector3f pos;     // Position (x, y, z)
    Eigen::Vector3f normal;  // Surface normal (nx, ny, nz)
};

}  // namespace transmuter::geometry

#endif
