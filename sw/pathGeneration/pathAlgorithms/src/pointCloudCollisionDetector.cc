#include "pointCloudCollisionDetector.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace transmuter::pathgenerator::algorithms
{

namespace
{

constexpr float MINIMUM_GRID_SIZE = 1.0f;

PointCloudBounds calculateBoundsFromVertices(const std::vector<geometry::Vertex>& vertices)
{
    PointCloudBounds bounds;
    if (vertices.empty())
    {
        return bounds;
    }

    bounds.min = vertices.front().pos;
    bounds.max = vertices.front().pos;
    bounds.valid = true;

    for (const auto& vertex : vertices)
    {
        bounds.min = bounds.min.cwiseMin(vertex.pos);
        bounds.max = bounds.max.cwiseMax(vertex.pos);
    }

    return bounds;
}

float estimateInterpolationRadius(const std::vector<geometry::Vertex>& vertices)
{
    if (vertices.size() < 2)
    {
        return 0.0f;
    }

    // Sampling a bounded subset keeps construction inexpensive for imported high-density clouds.
    const size_t sampleCount = std::min<size_t>(vertices.size(), 256);
    const size_t sampleStride = std::max<size_t>(vertices.size() / sampleCount, 1);
    std::vector<float> nearestDistances;
    nearestDistances.reserve(sampleCount);

    for (size_t sample = 0, index = 0; sample < sampleCount && index < vertices.size();
         ++sample, index += sampleStride)
    {
        float nearestSquared = std::numeric_limits<float>::max();
        for (size_t other = 0; other < vertices.size(); ++other)
        {
            if (other == index)
            {
                continue;
            }
            const float distanceSquared = (vertices[index].pos - vertices[other].pos).squaredNorm();
            // Shape generators commonly duplicate vertices along face boundaries. Ignore those
            // duplicates so they do not collapse the inferred distance between surface samples.
            if (distanceSquared > 0.0f)
            {
                nearestSquared = std::min(nearestSquared, distanceSquared);
            }
        }

        if (nearestSquared < std::numeric_limits<float>::max() && nearestSquared > 0.0f)
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

bool isInsideBounds(const PointCloudBounds& bounds, const Eigen::Vector3f& point, float tolerance)
{
    return bounds.valid && (point.array() >= (bounds.min.array() - tolerance)).all() &&
           (point.array() <= (bounds.max.array() + tolerance)).all();
}

}  // namespace

PointCloudCollisionDetector::PointCloudCollisionDetector(const geometry::PointCloud& fixedCloud)
{
    fixedCloud.copyTo(m_fixedVertices);
    m_fixedBounds = calculateBoundsFromVertices(m_fixedVertices);
    m_interpolationRadius = estimateInterpolationRadius(m_fixedVertices);
    m_gridCellSize = std::max(m_interpolationRadius * 2.0f, MINIMUM_GRID_SIZE);

    for (size_t index = 0; index < m_fixedVertices.size(); ++index)
    {
        m_xyGrid[gridKey(m_fixedVertices[index].pos.head<2>())].push_back(index);
    }
}

PointCloudBounds PointCloudCollisionDetector::calculateBounds(const geometry::PointCloud& cloud)
{
    std::vector<geometry::Vertex> vertices;
    cloud.copyTo(vertices);
    return calculateBoundsFromVertices(vertices);
}

size_t PointCloudCollisionDetector::GridKeyHash::operator()(const GridKey& key) const
{
    const auto x = static_cast<uint32_t>(key.x);
    const auto y = static_cast<uint32_t>(key.y);
    return static_cast<size_t>((static_cast<uint64_t>(x) << 32U) ^ y);
}

PointCloudCollisionDetector::GridKey PointCloudCollisionDetector::gridKey(
    const Eigen::Vector2f& point) const
{
    return GridKey{static_cast<int>(std::floor(point.x() / m_gridCellSize)),
                   static_cast<int>(std::floor(point.y() / m_gridCellSize))};
}

bool PointCloudCollisionDetector::approximatelyContains(const std::vector<geometry::Vertex>& cloud,
                                                        const PointCloudBounds& bounds,
                                                        const Eigen::Vector3f& point,
                                                        float tolerance) const
{
    if (cloud.empty() || !isInsideBounds(bounds, point, tolerance))
    {
        return false;
    }

    const geometry::Vertex* nearest = nullptr;
    float nearestSquared = std::numeric_limits<float>::max();
    for (const auto& vertex : cloud)
    {
        const float distanceSquared = (point - vertex.pos).squaredNorm();
        if (distanceSquared < nearestSquared)
        {
            nearest = &vertex;
            nearestSquared = distanceSquared;
        }
    }

    if (!nearest)
    {
        return false;
    }
    if (nearestSquared <= tolerance * tolerance)
    {
        return true;
    }

    // Closed point clouds carry outward normals. The nearest surface element therefore gives a
    // useful meshless inside/outside approximation away from the interpolated surface band.
    const float normalLength = nearest->normal.norm();
    return normalLength > 0.0f && (point - nearest->pos).dot(nearest->normal / normalLength) < 0.0f;
}

bool PointCloudCollisionDetector::intersects(const geometry::PointCloud& movingCloud,
                                             const Eigen::Affine3f& movingToFixed,
                                             float additionalTolerance) const
{
    std::vector<geometry::Vertex> movingVertices;
    movingCloud.copyTo(movingVertices);
    if (m_fixedVertices.empty() || movingVertices.empty())
    {
        return false;
    }

    const float movingInterpolationRadius = estimateInterpolationRadius(movingVertices);
    const float tolerance = std::max(
        0.0f, std::max(m_interpolationRadius, movingInterpolationRadius) + additionalTolerance);

    std::vector<geometry::Vertex> transformedMovingVertices = movingVertices;
    for (auto& vertex : transformedMovingVertices)
    {
        vertex.pos = movingToFixed * vertex.pos;
        vertex.normal = movingToFixed.linear() * vertex.normal;
    }
    const PointCloudBounds movingBounds = calculateBoundsFromVertices(transformedMovingVertices);

    for (const auto& movingVertex : transformedMovingVertices)
    {
        if (approximatelyContains(m_fixedVertices, m_fixedBounds, movingVertex.pos, tolerance))
        {
            return true;
        }
    }

    for (const auto& fixedVertex : m_fixedVertices)
    {
        if (approximatelyContains(transformedMovingVertices, movingBounds, fixedVertex.pos,
                                  tolerance))
        {
            return true;
        }
    }

    return false;
}

float PointCloudCollisionDetector::collisionFreeVerticalToolTipZ(const Eigen::Vector2f& toolCenter,
                                                                 float requestedTipZ,
                                                                 float toolRadius,
                                                                 float toolMinimumLocalZ,
                                                                 float clearance) const
{
    const float queryRadius = toolRadius + m_interpolationRadius;
    const float queryRadiusSquared = queryRadius * queryRadius;
    const GridKey minimumKey = gridKey(toolCenter - Eigen::Vector2f::Constant(queryRadius));
    const GridKey maximumKey = gridKey(toolCenter + Eigen::Vector2f::Constant(queryRadius));

    float collisionFreeZ = requestedTipZ;
    for (int x = minimumKey.x; x <= maximumKey.x; ++x)
    {
        for (int y = minimumKey.y; y <= maximumKey.y; ++y)
        {
            const auto cell = m_xyGrid.find(GridKey{x, y});
            if (cell == m_xyGrid.end())
            {
                continue;
            }

            for (const size_t index : cell->second)
            {
                const auto& obstaclePoint = m_fixedVertices[index].pos;
                if ((obstaclePoint.head<2>() - toolCenter).squaredNorm() <= queryRadiusSquared)
                {
                    collisionFreeZ =
                        std::max(collisionFreeZ, obstaclePoint.z() + m_interpolationRadius +
                                                     clearance - toolMinimumLocalZ);
                }
            }
        }
    }

    return collisionFreeZ;
}

float PointCloudCollisionDetector::interpolationRadius() const
{
    return m_interpolationRadius;
}

}  // namespace transmuter::pathgenerator::algorithms
