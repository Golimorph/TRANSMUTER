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
    Eigen::Vector3d min = Eigen::Vector3d::Zero();
    Eigen::Vector3d max = Eigen::Vector3d::Zero();
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

    /** Return whether a point expressed in this geometry's local coordinates is inside it. */
    virtual bool isInside(const Eigen::Vector3d& point) const = 0;

    /** Return this geometry clipped by another geometry, in this geometry's representation. */
    virtual std::shared_ptr<GeometryIf> intersection(const GeometryIf& geometry, const Eigen::Affine3d& geometryToLocalTransform = Eigen::Affine3d::Identity()) const;

    /** Return whether geometry at the supplied pose intersects this object. */
    virtual bool intersects(const GeometryIf& geometry, const Eigen::Affine3d& geometryToLocalTransform = Eigen::Affine3d::Identity()) const = 0;

    /** Add geometry after mapping its coordinates into this object's local frame. */
    virtual void combine(const GeometryIf& geometry, const Eigen::Affine3d& geometryToLocalTransform = Eigen::Affine3d::Identity()) = 0;

    /** Subtract cuttingGeometry at a pose mapping its local coordinates into this object. */
    virtual void cut(const GeometryIf& cuttingGeometry, const Eigen::Affine3d& cuttingToThis = Eigen::Affine3d::Identity()) = 0;

    /** Subtract multiple poses in one operation so implementations can optimize a tool sweep. */
    virtual void cut(const GeometryIf& cuttingGeometry, const std::vector<Eigen::Affine3d>& cuttingToThis)
    {
        for (const auto& transform : cuttingToThis)
        {
            cut(cuttingGeometry, transform);
        }
    }
    virtual GeometryBounds bounds() const = 0;
    virtual double maximumDistanceFromAxis(const Eigen::Vector3d& axisPoint, const Eigen::Vector3d& axisDirection) const = 0;
    virtual std::shared_ptr<GeometryIf> clone() const = 0;
    /** Material samples for geometric simulation, independent of rendering storage. */
    virtual std::vector<Eigen::Vector3d> sampleMaterial(double spacing) const;
    virtual void copyVisualizationVertices(std::vector<Vertex>& output) const = 0;
};

}  // namespace transmuter::geometry

#endif  // TRANSMUTER_GEOMETRY_GEOMETRY_IF_H
