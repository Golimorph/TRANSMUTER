#include "../inc/pointCloud.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>

#include "trace.h"

namespace transmuter::geometry
{

namespace
{

struct GridKey
{
    int x;
    int y;
    int z;

    bool operator==(const GridKey& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct GridKeyHash
{
    size_t operator()(const GridKey& key) const
    {
        const auto x = static_cast<uint32_t>(key.x);
        const auto y = static_cast<uint32_t>(key.y);
        const auto z = static_cast<uint32_t>(key.z);
        return static_cast<size_t>(x) * 73856093U ^ static_cast<size_t>(y) * 19349663U ^ static_cast<size_t>(z) * 83492791U;
    }
};

using CuttingGrid = std::unordered_map<GridKey, std::vector<size_t>, GridKeyHash>;

struct CuttingIndex
{
    CuttingGrid grid;
    std::vector<Eigen::Affine3d> pointCloudToCutting;
    double cellSize;
};

GridKey gridKey(const Eigen::Vector3d& point, double cellSize)
{
    return GridKey{static_cast<int>(std::floor(point.x() / cellSize)),
        static_cast<int>(std::floor(point.y() / cellSize)), static_cast<int>(std::floor(point.z() / cellSize))};
}

GeometryBounds transformedBounds(const GeometryBounds& bounds, const Eigen::Affine3d& transform)
{
    GeometryBounds result;
    for (int x = 0; x < 2; ++x)
    {
        for (int y = 0; y < 2; ++y)
        {
            for (int z = 0; z < 2; ++z)
            {
                const Eigen::Vector3d corner(x ? bounds.max.x() : bounds.min.x(),
                    y ? bounds.max.y() : bounds.min.y(), z ? bounds.max.z() : bounds.min.z());
                const Eigen::Vector3d transformed = transform * corner;
                result.min = result.isValid ? result.min.cwiseMin(transformed) : transformed;
                result.max = result.isValid ? result.max.cwiseMax(transformed) : transformed;
                result.isValid = true;
            }
        }
    }
    return result;
}

void insertCuttingBounds(CuttingGrid& grid, const GeometryBounds& bounds, double cellSize, size_t transformIndex)
{
    const GridKey minimum = gridKey(bounds.min, cellSize);
    const GridKey maximum = gridKey(bounds.max, cellSize);
    for (int x = minimum.x; x <= maximum.x; ++x)
    {
        for (int y = minimum.y; y <= maximum.y; ++y)
        {
            for (int z = minimum.z; z <= maximum.z; ++z)
            {
                grid[GridKey{x, y, z}].push_back(transformIndex);
            }
        }
    }
}

double cuttingGridCellSize(const GeometryBounds& bounds)
{
    const Eigen::Vector3d extents = bounds.max - bounds.min;
    double smallestExtent = std::numeric_limits<double>::max();
    for (const double extent : {extents.x(), extents.y(), extents.z()})
    {
        if (extent > 0.0)
        {
            smallestExtent = std::min(smallestExtent, extent);
        }
    }
    return smallestExtent < std::numeric_limits<double>::max() ? smallestExtent : 1.0;
}

CuttingIndex buildCuttingIndex(const GeometryBounds& cuttingBounds, const std::vector<Eigen::Affine3d>& cuttingToPointCloud)
{
    CuttingIndex result{
        .grid = {},
        .pointCloudToCutting = {},
        .cellSize = cuttingGridCellSize(cuttingBounds),
    };
    result.pointCloudToCutting.reserve(cuttingToPointCloud.size());
    for (size_t index = 0; index < cuttingToPointCloud.size(); ++index)
    {
        insertCuttingBounds(result.grid, transformedBounds(cuttingBounds, cuttingToPointCloud[index]),
            result.cellSize, index);
        result.pointCloudToCutting.push_back(cuttingToPointCloud[index].inverse());
    }
    return result;
}

bool isInsideAnyCuttingGeometry(const GeometryIf& cuttingGeometry, const CuttingIndex& index, const Eigen::Vector3d& point)
{
    const auto candidates = index.grid.find(gridKey(point, index.cellSize));
    return candidates != index.grid.end() &&
           std::any_of(candidates->second.begin(), candidates->second.end(),
               [&](size_t transformIndex) {
                   return cuttingGeometry.isInside(index.pointCloudToCutting[transformIndex] * point);
               });
}

}  // namespace

bool PointCloud::isInside(const Eigen::Vector3d& point) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_vertices.empty())
    {
        return false;
    }

    const double spacing = minimumPointSpacingLocked();
    const double spacingSquared = spacing * spacing;
    return std::any_of(m_vertices.begin(), m_vertices.end(),
        [&](const Vertex& vertex) { return (vertex.pos - point).squaredNorm() <= spacingSquared; });
}

bool PointCloud::intersects(const GeometryIf& geometry, const Eigen::Affine3d& geometryToLocalTransform) const
{
    if (&geometry == this)
    {
        return !isEmpty();
    }

    const Eigen::Affine3d pointCloudToGeometry = geometryToLocalTransform.inverse();
    std::lock_guard<std::mutex> lock(m_mutex);
    return std::any_of(m_vertices.begin(), m_vertices.end(), [&](const Vertex& vertex)
        { return geometry.isInside(pointCloudToGeometry * vertex.pos); });
}

void PointCloud::combine(const GeometryIf& geometry, const Eigen::Affine3d& geometryToLocalTransform)
{
    std::vector<Vertex> addedVertices;
    geometry.copyVisualizationVertices(addedVertices);
    for (auto& vertex : addedVertices)
    {
        vertex.pos = geometryToLocalTransform * vertex.pos;
        vertex.normal = geometryToLocalTransform.linear() * vertex.normal;
        if (!vertex.normal.isZero())
        {
            vertex.normal.normalize();
        }
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    m_vertices.insert(m_vertices.end(), addedVertices.begin(), addedVertices.end());
    m_minimumPointSpacing.reset();
}

void PointCloud::cut(const GeometryIf& cuttingGeometry, const Eigen::Affine3d& cuttingToThis)
{
    if (&cuttingGeometry == this)
    {
        clear();
        return;
    }

    const Eigen::Affine3d thisToCutting = cuttingToThis.inverse();
    eraseIf([&](const Vertex& vertex)
        { return cuttingGeometry.isInside(thisToCutting * vertex.pos); });
}

void PointCloud::cut(const GeometryIf& cuttingGeometry, const std::vector<Eigen::Affine3d>& cuttingToThis)
{
    if (cuttingToThis.empty())
    {
        return;
    }
    if (&cuttingGeometry == this)
    {
        clear();
        return;
    }

    const GeometryBounds cuttingBounds = cuttingGeometry.bounds();
    if (!cuttingBounds.isValid)
    {
        return;
    }

    const CuttingIndex cuttingIndex = buildCuttingIndex(cuttingBounds, cuttingToThis);
    eraseIf([&](const Vertex& vertex)
        { return isInsideAnyCuttingGeometry(cuttingGeometry, cuttingIndex, vertex.pos); });
}

double PointCloud::minimumPointSpacingLocked() const
{
    if (m_minimumPointSpacing)
    {
        return *m_minimumPointSpacing;
    }

    double minimumSquared = std::numeric_limits<double>::max();
    for (size_t first = 0; first < m_vertices.size(); ++first)
    {
        for (size_t second = first + 1; second < m_vertices.size(); ++second)
        {
            const double distance = (m_vertices[first].pos - m_vertices[second].pos).squaredNorm();
            if (distance > 0.0)
            {
                minimumSquared = std::min(minimumSquared, distance);
            }
        }
    }

    m_minimumPointSpacing = minimumSquared < std::numeric_limits<double>::max() ? std::sqrt(minimumSquared) : 0.0;
    return *m_minimumPointSpacing;
}

GeometryBounds PointCloud::bounds() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    GeometryBounds result;
    if (m_vertices.empty())
    {
        return result;
    }

    result.min = m_vertices.front().pos;
    result.max = m_vertices.front().pos;
    result.isValid = true;
    for (const auto& vertex : m_vertices)
    {
        result.min = result.min.cwiseMin(vertex.pos);
        result.max = result.max.cwiseMax(vertex.pos);
    }
    return result;
}

double PointCloud::maximumDistanceFromAxis(const Eigen::Vector3d& axisPoint, const Eigen::Vector3d& axisDirection) const
{
    const double axisLengthSquared = axisDirection.squaredNorm();
    if (axisLengthSquared <= 0.0)
    {
        ERROR("Geometry axis direction must not be zero.");
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    double maximumDistanceSquared = 0.0;
    for (const auto& vertex : m_vertices)
    {
        const Eigen::Vector3d offset = vertex.pos - axisPoint;
        const double axialDistance = offset.dot(axisDirection);
        const double axialDistanceSquared = axialDistance * axialDistance / axisLengthSquared;
        maximumDistanceSquared = std::max(maximumDistanceSquared, offset.squaredNorm() - axialDistanceSquared);
    }
    return std::sqrt(std::max(maximumDistanceSquared, 0.0));
}

}  // namespace transmuter::geometry
