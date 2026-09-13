#ifndef TRANSMUTER_GEOMETRY_GEOMETRY_IF_H
#define TRANSMUTER_GEOMETRY_GEOMETRY_IF_H

#include <Eigen/Geometry>
#include <memory>
#include <vector>

#include "vertex.h"

namespace transmuter::geometry
{

struct GeometryBounds
{
    Eigen::Vector3f min = Eigen::Vector3f::Zero();
    Eigen::Vector3f max = Eigen::Vector3f::Zero();
    bool isValid = false;
};

/**
 * Representation-independent access to a three-dimensional object.
 *
 * Algorithms use geometric queries rather than inspecting the underlying point, mesh, voxel, or
 * implicit representation. Visualization vertices are an interchange format for the current
 * renderer; they do not prescribe how the object is stored.
 */
class GeometryIf
{
   public:
    virtual ~GeometryIf() = default;

    virtual GeometryBounds bounds() const = 0;
    virtual float maximumDistanceFromAxis(const Eigen::Vector3f& axisPoint,
                                          const Eigen::Vector3f& axisDirection) const = 0;
    virtual std::shared_ptr<GeometryIf> clone() const = 0;
    virtual void copyVisualizationVertices(std::vector<Vertex>& output) const = 0;
};

}  // namespace transmuter::geometry

#endif  // TRANSMUTER_GEOMETRY_GEOMETRY_IF_H
