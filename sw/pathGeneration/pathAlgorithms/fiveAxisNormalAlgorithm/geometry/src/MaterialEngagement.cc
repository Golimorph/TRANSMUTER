#include "MaterialEngagement.h"
namespace transmuter::pathgenerator::algorithms
{
std::vector<Eigen::Vector3d> MaterialEngagement::sample(const geometry::GeometryIf& stock, const geometry::GeometryIf& tool, const Eigen::Affine3d& pose, double spacing)
{
    std::vector<Eigen::Vector3d> result;
    if (!stock.intersects(tool, pose))
    {
        return result;
    }
    const Eigen::Affine3d inverse = pose.inverse();
    for (const auto& point : stock.sampleMaterial(spacing))
    {
        if (tool.isInside(inverse * point))
        {
            result.push_back(point);
        }
    }
    return result;
}
}  // namespace transmuter::pathgenerator::algorithms
