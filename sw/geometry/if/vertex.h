#ifndef TRANSMUTER_GEOMETRY_VERTEX_H
#define TRANSMUTER_GEOMETRY_VERTEX_H

#include <Eigen/Dense>
#include <cmath>

#include "trace.h"

namespace transmuter::geometry
{

/**
 * A sampled position and normal used for geometry presentation and tool poses.
 */
struct Vertex
{
    Vertex() : pos(Eigen::Vector3d::Zero()), normal(Eigen::Vector3d::UnitZ())
    {
    }

    Vertex(const Eigen::Vector3d& position, const Eigen::Vector3d& direction)
        : pos(position), normal(direction)
    {
        const double normalLength = normal.norm();
        if (!pos.allFinite() || !normal.allFinite() || !std::isfinite(normalLength) || normalLength <= 1e-12)
        {
            ERROR("Vertex position and direction must be finite, with a nonzero direction.");
        }
    }

    Eigen::Vector3d pos;     // [x, y, z]
    Eigen::Vector3d normal;  // [nx, ny, nz]
};

}  // namespace transmuter::geometry
#endif  // TRANSMUTER_GEOMETRY_VERTEX_H
