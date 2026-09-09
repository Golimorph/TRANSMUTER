#include "materialRemoval.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>

namespace transmuter::geometry
{

namespace
{

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

bool isVertical(const Vertex& pose)
{
    const float normalLength = pose.normal.norm();
    return normalLength > 0.0f &&
           pose.normal.dot(Eigen::Vector3f::UnitZ()) / normalLength > 0.9999f;
}

}  // namespace

MaterialRemoval::MaterialRemoval(const PointCloud& toolModel)
{
    const size_t toolVertexCount = toolModel.size();
    if (toolVertexCount == 0)
    {
        return;
    }

    m_toolMinimumLocalZ = std::numeric_limits<float>::max();
    m_toolMaximumLocalZ = std::numeric_limits<float>::lowest();
    for (size_t index = 0; index < toolVertexCount; ++index)
    {
        const Vertex vertex = toolModel.getVertex(index);
        m_toolRadius = std::max(m_toolRadius, vertex.pos.head<2>().norm());
        m_toolMinimumLocalZ = std::min(m_toolMinimumLocalZ, vertex.pos.z());
        m_toolMaximumLocalZ = std::max(m_toolMaximumLocalZ, vertex.pos.z());
    }
}

size_t MaterialRemoval::apply(PointCloud& workpiece,
                              const std::vector<Vertex>& toolPoses) const
{
    if (toolPoses.empty() || m_toolRadius <= 0.0f ||
        m_toolMaximumLocalZ <= m_toolMinimumLocalZ)
    {
        return 0;
    }

    if (std::all_of(toolPoses.begin(), toolPoses.end(), isVertical))
    {
        return removeVerticalMaterial(workpiece, toolPoses);
    }
    return removeOrientedMaterial(workpiece, toolPoses);
}

size_t MaterialRemoval::removeVerticalMaterial(
    PointCloud& workpiece, const std::vector<Vertex>& toolPoses) const
{
    const float cellSize = m_toolRadius;
    std::unordered_map<GridKey, std::vector<size_t>, GridKeyHash> pathGrid;
    for (size_t index = 0; index < toolPoses.size(); ++index)
    {
        const auto& point = toolPoses[index].pos;
        const GridKey key{static_cast<int>(std::floor(point.x() / cellSize)),
                          static_cast<int>(std::floor(point.y() / cellSize))};
        pathGrid[key].push_back(index);
    }

    const float radiusSquared = m_toolRadius * m_toolRadius;
    return workpiece.eraseIf(
        [&](const Vertex& workpieceVertex)
        {
            const GridKey centerKey{
                static_cast<int>(std::floor(workpieceVertex.pos.x() / cellSize)),
                static_cast<int>(std::floor(workpieceVertex.pos.y() / cellSize))};

            for (int x = centerKey.x - 1; x <= centerKey.x + 1; ++x)
            {
                for (int y = centerKey.y - 1; y <= centerKey.y + 1; ++y)
                {
                    const auto cell = pathGrid.find(GridKey{x, y});
                    if (cell == pathGrid.end())
                    {
                        continue;
                    }

                    for (const size_t pathIndex : cell->second)
                    {
                        const auto& toolPosition = toolPoses[pathIndex].pos;
                        const float dx = workpieceVertex.pos.x() - toolPosition.x();
                        const float dy = workpieceVertex.pos.y() - toolPosition.y();
                        const float localZ = workpieceVertex.pos.z() - toolPosition.z();
                        if (dx * dx + dy * dy <= radiusSquared &&
                            localZ >= m_toolMinimumLocalZ && localZ <= m_toolMaximumLocalZ)
                        {
                            return true;
                        }
                    }
                }
            }
            return false;
        });
}

size_t MaterialRemoval::removeOrientedMaterial(
    PointCloud& workpiece, const std::vector<Vertex>& toolPoses) const
{
    const float radiusSquared = m_toolRadius * m_toolRadius;
    return workpiece.eraseIf(
        [&](const Vertex& workpieceVertex)
        {
            for (const auto& pose : toolPoses)
            {
                if (pose.normal.isZero())
                {
                    continue;
                }

                const Eigen::Quaternionf toolRotation = Eigen::Quaternionf::FromTwoVectors(
                    Eigen::Vector3f::UnitZ(), pose.normal.normalized());
                const Eigen::Vector3f localPoint =
                    toolRotation.conjugate() * (workpieceVertex.pos - pose.pos);
                if (localPoint.head<2>().squaredNorm() <= radiusSquared &&
                    localPoint.z() >= m_toolMinimumLocalZ &&
                    localPoint.z() <= m_toolMaximumLocalZ)
                {
                    return true;
                }
            }
            return false;
        });
}

}  // namespace transmuter::geometry
