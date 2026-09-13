#include "pointCloudCollisionDetector.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_map>

#include "trace.h"
#include "verticalToolClearance.h"

namespace transmuter::geometry
{

namespace
{

constexpr float MINIMUM_GRID_SIZE = 1.0f;

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
    size_t operator()(const GridKey& key) const
    {
        const auto x = static_cast<uint32_t>(key.x);
        const auto y = static_cast<uint32_t>(key.y);
        return static_cast<size_t>((static_cast<uint64_t>(x) << 32U) ^ y);
    }
};

GridKey gridKey(const Eigen::Vector2f& point, float cellSize)
{
    return GridKey{static_cast<int>(std::floor(point.x() / cellSize)),
                   static_cast<int>(std::floor(point.y() / cellSize))};
}

GeometryBounds calculateBoundsFromVertices(const std::vector<Vertex>& vertices)
{
    GeometryBounds bounds;
    if (vertices.empty())
    {
        return bounds;
    }

    bounds.min = vertices.front().pos;
    bounds.max = vertices.front().pos;
    bounds.isValid = true;

    for (const auto& vertex : vertices)
    {
        bounds.min = bounds.min.cwiseMin(vertex.pos);
        bounds.max = bounds.max.cwiseMax(vertex.pos);
    }

    return bounds;
}

float nearestDistinctDistanceSquared(const std::vector<Vertex>& vertices, size_t index)
{
    float nearest = std::numeric_limits<float>::max();
    for (size_t other = 0; other < vertices.size(); ++other)
    {
        if (other == index)
        {
            continue;
        }

        const float distance = (vertices[index].pos - vertices[other].pos).squaredNorm();
        if (distance > 0.0f)
        {
            nearest = std::min(nearest, distance);
        }
    }
    return nearest;
}

float estimateInterpolationRadius(const std::vector<Vertex>& vertices)
{
    if (vertices.size() < 2)
    {
        return 0.0f;
    }

    std::vector<float> nearestDistances;
    const size_t sampleCount = std::min<size_t>(vertices.size(), 256);
    nearestDistances.reserve(sampleCount);
    const size_t stride = std::max<size_t>(vertices.size() / sampleCount, 1);
    for (size_t index = 0; index < vertices.size() && nearestDistances.size() < sampleCount;
         index += stride)
    {
        const float nearestSquared = nearestDistinctDistanceSquared(vertices, index);
        if (nearestSquared < std::numeric_limits<float>::max())
        {
            nearestDistances.push_back(std::sqrt(nearestSquared));
        }
    }

    if (nearestDistances.empty())
    {
        return 0.0f;
    }

    const auto middle = nearestDistances.begin() + nearestDistances.size() / 2;
    std::nth_element(nearestDistances.begin(), middle, nearestDistances.end());
    return *middle * 0.5f;
}

struct NearestVertex
{
    const Vertex* vertex = nullptr;
    float distanceSquared = std::numeric_limits<float>::max();
};

NearestVertex findNearestVertex(const std::vector<Vertex>& cloud,
                                const Eigen::Vector3f& point)
{
    NearestVertex nearest;
    for (const auto& vertex : cloud)
    {
        const float distanceSquared = (point - vertex.pos).squaredNorm();
        if (distanceSquared < nearest.distanceSquared)
        {
            nearest = NearestVertex{&vertex, distanceSquared};
        }
    }
    return nearest;
}

bool isInsideBounds(const GeometryBounds& bounds, const Eigen::Vector3f& point, float tolerance)
{
    return bounds.isValid && (point.array() >= (bounds.min.array() - tolerance)).all() &&
           (point.array() <= (bounds.max.array() + tolerance)).all();
}

bool approximatelyContains(const std::vector<Vertex>& cloud, const GeometryBounds& bounds,
                           const Eigen::Vector3f& point, float tolerance)
{
    if (cloud.empty() || !isInsideBounds(bounds, point, tolerance))
    {
        return false;
    }

    const NearestVertex nearest = findNearestVertex(cloud, point);
    if (nearest.distanceSquared <= tolerance * tolerance)
    {
        return true;
    }

    // Outward normals provide a meshless inside/outside approximation away from the surface.
    const float normalLength = nearest.vertex->normal.norm();
    return normalLength > 0.0f &&
           (point - nearest.vertex->pos).dot(nearest.vertex->normal / normalLength) < 0.0f;
}

std::vector<Vertex> transformVertices(const std::vector<Vertex>& vertices,
                                      const Eigen::Affine3f& transform)
{
    std::vector<Vertex> transformed = vertices;
    for (auto& vertex : transformed)
    {
        vertex.pos = transform * vertex.pos;
        vertex.normal = transform.linear() * vertex.normal;
    }
    return transformed;
}

bool containsAny(const std::vector<Vertex>& container, const GeometryBounds& bounds,
                 const std::vector<Vertex>& candidates, float tolerance)
{
    return std::any_of(candidates.begin(), candidates.end(), [&](const Vertex& candidate)
                       { return approximatelyContains(container, bounds, candidate.pos,
                                                      tolerance); });
}

class PointCloudVerticalToolClearance final : public VerticalToolClearanceIf
{
   public:
    PointCloudVerticalToolClearance(const PointCloud& obstacle, const PointCloud& tool)
    {
        obstacle.copyTo(m_obstacleVertices);
        m_interpolationRadius = estimateInterpolationRadius(m_obstacleVertices);
        m_gridCellSize = std::max(m_interpolationRadius * 2.0f, MINIMUM_GRID_SIZE);

        const GeometryBounds toolBounds = tool.bounds();
        m_toolRadius = tool.maximumDistanceFromAxis(Eigen::Vector3f::Zero(),
                                                    Eigen::Vector3f::UnitZ());
        m_toolMinimumLocalZ = toolBounds.min.z();

        for (size_t index = 0; index < m_obstacleVertices.size(); ++index)
        {
            m_xyGrid[gridKey(m_obstacleVertices[index].pos.head<2>(), m_gridCellSize)]
                .push_back(index);
        }
    }

    Eigen::Vector3f liftToAvoidCollision(
        const Eigen::Vector3f& requestedToolPosition, float clearance) const override
    {
        const Eigen::Vector2f toolCenter = requestedToolPosition.head<2>();
        const float queryRadius = m_toolRadius + m_interpolationRadius;
        const float queryRadiusSquared = queryRadius * queryRadius;
        const GridKey minimumKey =
            gridKey(toolCenter - Eigen::Vector2f::Constant(queryRadius), m_gridCellSize);
        const GridKey maximumKey =
            gridKey(toolCenter + Eigen::Vector2f::Constant(queryRadius), m_gridCellSize);

        Eigen::Vector3f collisionFreePosition = requestedToolPosition;
        for (int x = minimumKey.x; x <= maximumKey.x; ++x)
        {
            for (int y = minimumKey.y; y <= maximumKey.y; ++y)
            {
                liftAboveCell(GridKey{x, y}, toolCenter, queryRadiusSquared, clearance,
                              collisionFreePosition);
            }
        }
        return collisionFreePosition;
    }

    float geometryTolerance() const override
    {
        return m_interpolationRadius;
    }

   private:
    void liftAboveCell(const GridKey& key, const Eigen::Vector2f& toolCenter,
                       float queryRadiusSquared, float clearance,
                       Eigen::Vector3f& toolPosition) const
    {
        const auto cell = m_xyGrid.find(key);
        if (cell == m_xyGrid.end())
        {
            return;
        }

        for (const size_t index : cell->second)
        {
            const Eigen::Vector3f& obstacle = m_obstacleVertices[index].pos;
            if ((obstacle.head<2>() - toolCenter).squaredNorm() <= queryRadiusSquared)
            {
                toolPosition.z() = std::max(
                    toolPosition.z(), obstacle.z() + m_interpolationRadius + clearance -
                                          m_toolMinimumLocalZ);
            }
        }
    }

    std::vector<Vertex> m_obstacleVertices;
    float m_interpolationRadius = 0.0f;
    float m_gridCellSize = 1.0f;
    float m_toolRadius = 0.0f;
    float m_toolMinimumLocalZ = 0.0f;
    std::unordered_map<GridKey, std::vector<size_t>, GridKeyHash> m_xyGrid;
};

}  // namespace

PointCloudCollisionDetector::PointCloudCollisionDetector(const PointCloud& fixedCloud)
{
    fixedCloud.copyTo(m_fixedVertices);
    m_fixedBounds = calculateBoundsFromVertices(m_fixedVertices);
    m_interpolationRadius = estimateInterpolationRadius(m_fixedVertices);
}

bool PointCloudCollisionDetector::intersects(const GeometryIf& movingGeometry,
                                             const Eigen::Affine3f& movingToFixed,
                                             float additionalTolerance) const
{
    const auto* movingCloud = dynamic_cast<const PointCloud*>(&movingGeometry);
    if (!movingCloud)
    {
        ERROR("Point-cloud collision detection requires point-cloud geometry.");
    }

    std::vector<Vertex> movingVertices;
    movingCloud->copyTo(movingVertices);
    if (m_fixedVertices.empty() || movingVertices.empty())
    {
        return false;
    }

    const float movingInterpolationRadius = estimateInterpolationRadius(movingVertices);
    const float tolerance = std::max(
        0.0f, std::max(m_interpolationRadius, movingInterpolationRadius) + additionalTolerance);

    const std::vector<Vertex> transformedMovingVertices =
        transformVertices(movingVertices, movingToFixed);
    const GeometryBounds movingBounds = calculateBoundsFromVertices(transformedMovingVertices);

    return containsAny(m_fixedVertices, m_fixedBounds, transformedMovingVertices, tolerance) ||
           containsAny(transformedMovingVertices, movingBounds, m_fixedVertices, tolerance);
}

std::unique_ptr<CollisionDetectorIf> createCollisionDetector(const GeometryIf& fixedGeometry)
{
    const auto* pointCloud = dynamic_cast<const PointCloud*>(&fixedGeometry);
    if (!pointCloud)
    {
        ERROR("No collision detector exists for this geometry representation.");
    }
    return std::make_unique<PointCloudCollisionDetector>(*pointCloud);
}

std::unique_ptr<VerticalToolClearanceIf> createVerticalToolClearance(
    const GeometryIf& obstacle, const GeometryIf& tool)
{
    const auto* pointCloudObstacle = dynamic_cast<const PointCloud*>(&obstacle);
    const auto* pointCloudTool = dynamic_cast<const PointCloud*>(&tool);
    if (!pointCloudObstacle || !pointCloudTool)
    {
        ERROR(
            "No vertical tool-clearance implementation exists for this geometry representation.");
    }
    return std::make_unique<PointCloudVerticalToolClearance>(*pointCloudObstacle,
                                                             *pointCloudTool);
}

}  // namespace transmuter::geometry
