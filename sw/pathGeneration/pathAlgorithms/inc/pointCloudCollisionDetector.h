#ifndef TRANSMUTER_PATHGEN_POINT_CLOUD_COLLISION_DETECTOR_H
#define TRANSMUTER_PATHGEN_POINT_CLOUD_COLLISION_DETECTOR_H

#include <Eigen/Geometry>
#include <cstddef>
#include <unordered_map>
#include <vector>

#include "pointCloud.h"

namespace transmuter::pathgenerator::algorithms
{

struct PointCloudBounds
{
    Eigen::Vector3f min = Eigen::Vector3f::Zero();
    Eigen::Vector3f max = Eigen::Vector3f::Zero();
    bool valid = false;
};

/**
 * Collision queries against a fixed point cloud.
 *
 * Point clouds contain samples rather than continuous triangles. Each fixed-cloud point is
 * therefore treated as a small surface element whose radius is inferred from the cloud spacing.
 * This closes the gaps between neighbouring samples without requiring a mesh.
 */
class PointCloudCollisionDetector
{
   public:
    explicit PointCloudCollisionDetector(const geometry::PointCloud& fixedCloud);

    static PointCloudBounds calculateBounds(const geometry::PointCloud& cloud);

    /**
     * @brief General point-cloud overlap test for a rigidly transformed moving cloud.
     */
    bool intersects(const geometry::PointCloud& movingCloud,
                    const Eigen::Affine3f& movingToFixed = Eigen::Affine3f::Identity(),
                    float additionalTolerance = 0.0f) const;

    /**
     * @brief Lift a vertical tool tip just enough to clear the fixed cloud.
     *
     * The tool is modelled as a vertical cutter with its axis at toolCenter and its lowest local
     * Z coordinate at the requested tip height. The query is intentionally conservative: nearby
     * surface samples are expanded by the inferred interpolation radius.
     */
    float collisionFreeVerticalToolTipZ(const Eigen::Vector2f& toolCenter, float requestedTipZ,
                                        float toolRadius, float toolMinimumLocalZ,
                                        float clearance = 0.0f) const;

    float interpolationRadius() const;

   private:
    struct GridKey
    {
        int x;
        int y;

        bool operator==(const GridKey& other) const
        {
            return x == other.x && y == other.y;
        }
    };

    struct GridKeyHash
    {
        size_t operator()(const GridKey& key) const;
    };

    GridKey gridKey(const Eigen::Vector2f& point) const;
    bool approximatelyContains(const std::vector<geometry::Vertex>& cloud,
                               const PointCloudBounds& bounds, const Eigen::Vector3f& point,
                               float tolerance) const;

    std::vector<geometry::Vertex> m_fixedVertices;
    PointCloudBounds m_fixedBounds;
    float m_interpolationRadius = 0.0f;
    float m_gridCellSize = 1.0f;
    std::unordered_map<GridKey, std::vector<size_t>, GridKeyHash> m_xyGrid;
};

}  // namespace transmuter::pathgenerator::algorithms

#endif  // TRANSMUTER_PATHGEN_POINT_CLOUD_COLLISION_DETECTOR_H
