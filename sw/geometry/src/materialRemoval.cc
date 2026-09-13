#include "materialRemoval.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <unordered_map>

#include "pointCloud.h"
#include "trace.h"

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

using PathGrid = std::unordered_map<GridKey, std::vector<size_t>, GridKeyHash>;

GridKey gridKey(const Eigen::Vector3f& point, float cellSize)
{
    return GridKey{static_cast<int>(std::floor(point.x() / cellSize)),
                   static_cast<int>(std::floor(point.y() / cellSize))};
}

PathGrid buildPathGrid(const std::vector<Vertex>& toolPoses, float cellSize)
{
    PathGrid grid;
    for (size_t index = 0; index < toolPoses.size(); ++index)
    {
        grid[gridKey(toolPoses[index].pos, cellSize)].push_back(index);
    }
    return grid;
}

bool isVertical(const Vertex& pose)
{
    const float normalLength = pose.normal.norm();
    return normalLength > 0.0f &&
           pose.normal.dot(Eigen::Vector3f::UnitZ()) / normalLength > 0.9999f;
}

class PointCloudMaterialRemoval final : public MaterialRemovalIf
{
   public:
    explicit PointCloudMaterialRemoval(const PointCloud& toolModel);

    void apply(GeometryIf& workpiece,
               const std::vector<Vertex>& toolPoses) const override;

   private:
    void removeVerticalMaterial(PointCloud& workpiece,
                                const std::vector<Vertex>& toolPoses) const;
    void removeOrientedMaterial(PointCloud& workpiece,
                                const std::vector<Vertex>& toolPoses) const;
    bool verticalSweepCutsVertex(const Vertex& workpieceVertex,
                                 const std::vector<Vertex>& toolPoses,
                                 const PathGrid& pathGrid, float cellSize,
                                 float radiusSquared) const;
    bool orientedToolCutsVertex(const Vertex& workpieceVertex,
                                const Vertex& toolPose, float radiusSquared) const;

    float m_toolRadius = 0.0f;
    float m_toolMinimumLocalZ = 0.0f;
    float m_toolMaximumLocalZ = 0.0f;
};

PointCloudMaterialRemoval::PointCloudMaterialRemoval(const PointCloud& toolModel)
{
    const GeometryBounds bounds = toolModel.bounds();
    if (!bounds.isValid)
    {
        return;
    }

    m_toolRadius = toolModel.maximumDistanceFromAxis(Eigen::Vector3f::Zero(),
                                                     Eigen::Vector3f::UnitZ());
    m_toolMinimumLocalZ = bounds.min.z();
    m_toolMaximumLocalZ = bounds.max.z();
}

void PointCloudMaterialRemoval::apply(GeometryIf& workpiece,
                                      const std::vector<Vertex>& toolPoses) const
{
    auto* pointCloud = dynamic_cast<PointCloud*>(&workpiece);
    if (!pointCloud)
    {
        ERROR("Point-cloud material removal requires a point-cloud workpiece.");
    }

    if (toolPoses.empty() || m_toolRadius <= 0.0f ||
        m_toolMaximumLocalZ <= m_toolMinimumLocalZ)
    {
        return;
    }

    if (std::all_of(toolPoses.begin(), toolPoses.end(), isVertical))
    {
        removeVerticalMaterial(*pointCloud, toolPoses);
        return;
    }
    removeOrientedMaterial(*pointCloud, toolPoses);
}

void PointCloudMaterialRemoval::removeVerticalMaterial(
    PointCloud& workpiece, const std::vector<Vertex>& toolPoses) const
{
    const float cellSize = m_toolRadius;
    const PathGrid pathGrid = buildPathGrid(toolPoses, cellSize);
    const float radiusSquared = m_toolRadius * m_toolRadius;
    workpiece.eraseIf(
        [&](const Vertex& vertex)
        { return verticalSweepCutsVertex(vertex, toolPoses, pathGrid, cellSize, radiusSquared); });
}

bool PointCloudMaterialRemoval::verticalSweepCutsVertex(
    const Vertex& workpieceVertex, const std::vector<Vertex>& toolPoses,
    const PathGrid& pathGrid, float cellSize, float radiusSquared) const
{
    const GridKey center = gridKey(workpieceVertex.pos, cellSize);
    for (int x = center.x - 1; x <= center.x + 1; ++x)
    {
        for (int y = center.y - 1; y <= center.y + 1; ++y)
        {
            const auto cell = pathGrid.find(GridKey{x, y});
            if (cell == pathGrid.end())
            {
                continue;
            }

            for (const size_t pathIndex : cell->second)
            {
                const Vertex& toolPose = toolPoses[pathIndex];
                const Eigen::Vector2f radialOffset =
                    workpieceVertex.pos.head<2>() - toolPose.pos.head<2>();
                const float localZ = workpieceVertex.pos.z() - toolPose.pos.z();
                if (radialOffset.squaredNorm() <= radiusSquared &&
                    localZ >= m_toolMinimumLocalZ && localZ <= m_toolMaximumLocalZ)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

void PointCloudMaterialRemoval::removeOrientedMaterial(
    PointCloud& workpiece, const std::vector<Vertex>& toolPoses) const
{
    const float radiusSquared = m_toolRadius * m_toolRadius;
    workpiece.eraseIf([&](const Vertex& vertex)
                      {
                          return std::any_of(
                              toolPoses.begin(), toolPoses.end(), [&](const Vertex& toolPose)
                              { return orientedToolCutsVertex(vertex, toolPose, radiusSquared); });
                      });
}

bool PointCloudMaterialRemoval::orientedToolCutsVertex(
    const Vertex& workpieceVertex, const Vertex& toolPose, float radiusSquared) const
{
    if (toolPose.normal.isZero())
    {
        return false;
    }

    const Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(
        Eigen::Vector3f::UnitZ(), toolPose.normal.normalized());
    const Eigen::Vector3f localPoint =
        rotation.conjugate() * (workpieceVertex.pos - toolPose.pos);
    return localPoint.head<2>().squaredNorm() <= radiusSquared &&
           localPoint.z() >= m_toolMinimumLocalZ && localPoint.z() <= m_toolMaximumLocalZ;
}

}  // namespace

std::unique_ptr<MaterialRemovalIf> createMaterialRemoval(const GeometryIf& workpiece,
                                                         const GeometryIf& toolModel)
{
    const auto* pointCloudWorkpiece = dynamic_cast<const PointCloud*>(&workpiece);
    const auto* pointCloudTool = dynamic_cast<const PointCloud*>(&toolModel);
    if (!pointCloudWorkpiece || !pointCloudTool)
    {
        ERROR("No material-removal implementation exists for these geometry representations.");
    }
    return std::make_unique<PointCloudMaterialRemoval>(*pointCloudTool);
}

}  // namespace transmuter::geometry
