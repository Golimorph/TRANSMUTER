#ifndef TRANSMUTER_GEOMETRY_COLLISION_DETECTION_H
#define TRANSMUTER_GEOMETRY_COLLISION_DETECTION_H

#include <Eigen/Geometry>
#include <memory>

#include "geometryIf.h"

namespace transmuter::geometry
{

/** Representation-independent collision queries against one fixed geometry. */
class CollisionDetectorIf
{
   public:
    virtual ~CollisionDetectorIf() = default;

    virtual bool intersects(
        const GeometryIf& movingGeometry,
        const Eigen::Affine3f& movingToFixed = Eigen::Affine3f::Identity(),
        float additionalTolerance = 0.0f) const = 0;
};

/** Creates collision queries appropriate for the fixed geometry representation. */
std::unique_ptr<CollisionDetectorIf> createCollisionDetector(const GeometryIf& fixedGeometry);

}  // namespace transmuter::geometry

#endif  // TRANSMUTER_GEOMETRY_COLLISION_DETECTION_H
