#ifndef TRANSMUTER_GEOMETRY_SHAPES_VERTEX_H
#define TRANSMUTER_GEOMETRY_SHAPES_VERTEX_H

#include <Eigen/Dense>

namespace transmuter::geometry
{

/**
 * Fundamental data unit for the 5-Axis engine.
 * Moved to shapes namespace to keep the geometry root clean.
 * * Using Eigen types ensures we can perform 4x4 matrix transforms
 * directly on the data points within the Cortex.
 */
struct Vertex
{
    Eigen::Vector3f pos;     // [x, y, z]
    Eigen::Vector3f normal;  // [nx, ny, nz]
};

}  // namespace transmuter::geometry
#endif