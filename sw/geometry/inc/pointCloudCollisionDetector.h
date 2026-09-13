#ifndef TRANSMUTER_GEOMETRY_POINT_CLOUD_COLLISION_DETECTOR_H
#define TRANSMUTER_GEOMETRY_POINT_CLOUD_COLLISION_DETECTOR_H

#include <Eigen/Geometry>
#include <vector>

#include "collisionDetection.h"
#include "pointCloud.h"

namespace transmuter::geometry
{

/**
 * Collision queries against a fixed point cloud.
 *
 * Point clouds contain samples rather than continuous triangles. Each fixed-cloud point is
 * therefore treated as a small surface element whose radius is inferred from the cloud spacing.
 * This closes the gaps between neighbouring samples without requiring a mesh.
 */
class PointCloudCollisionDetector final : public CollisionDetectorIf
{
   public:
    explicit PointCloudCollisionDetector(const PointCloud& fixedCloud);

    /**
     * @brief General point-cloud overlap test for a rigidly transformed moving cloud.
     */
    bool intersects(const GeometryIf& movingGeometry,
                    const Eigen::Affine3f& movingToFixed = Eigen::Affine3f::Identity(),
                    float additionalTolerance = 0.0f) const override;

   private:
    std::vector<Vertex> m_fixedVertices;
    GeometryBounds m_fixedBounds;
    float m_interpolationRadius = 0.0f;
};

}  // namespace transmuter::geometry

#endif  // TRANSMUTER_GEOMETRY_POINT_CLOUD_COLLISION_DETECTOR_H
