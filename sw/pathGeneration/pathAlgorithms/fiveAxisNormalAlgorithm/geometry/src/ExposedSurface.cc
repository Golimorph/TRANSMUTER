#include "ExposedSurface.h"

#include <array>
#include <map>
#include <set>
namespace transmuter::pathgenerator::algorithms
{
namespace
{
using Cell = std::array<int, 3>;
Cell cellAt(const Eigen::Vector3d& position, double spacing)
{
    const Eigen::Vector3i index = (position / spacing).array().round().cast<int>();
    return {index.x(), index.y(), index.z()};
}
Eigen::Vector3d exposedNormal(const Cell& cell, const std::set<Cell>& occupied)
{
    Eigen::Vector3d normal = Eigen::Vector3d::Zero(), fallback = Eigen::Vector3d::Zero();
    for (int axis = 0; axis < 3; ++axis)
    {
        for (int sign : {-1, 1})
        {
            auto neighbour = cell;
            neighbour[axis] += sign;
            if (occupied.count(neighbour))
            {
                continue;
            }
            normal[axis] += sign;
            if (fallback.isZero())
            {
                fallback[axis] = sign;
            }
        }
    }
    if (normal.isZero())
    {
        return fallback;
    }
    return normal.normalized();
}
}  // namespace
std::vector<geometry::Vertex> ExposedSurface::sample(const geometry::GeometryIf& workpiece, double materialSpacing, double passSpacing)
{
    const auto material = workpiece.sampleMaterial(materialSpacing);
    std::set<Cell> occupied;
    for (const auto& point : material)
    {
        occupied.insert(cellAt(point, materialSpacing));
    }
    std::map<Cell, geometry::Vertex> surface;
    for (const auto& point : material)
    {
        const Eigen::Vector3d normal = exposedNormal(cellAt(point, materialSpacing), occupied);
        if (!normal.isZero())
        {
            surface.try_emplace(cellAt(point, passSpacing), geometry::Vertex{point, normal});
        }
    }
    std::vector<geometry::Vertex> result;
    for (const auto& entry : surface)
    {
        result.push_back(entry.second);
    }
    return result;
}
}  // namespace transmuter::pathgenerator::algorithms
