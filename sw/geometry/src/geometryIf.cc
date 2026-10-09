#include "geometryIf.h"

#include <cmath>

#include "trace.h"

namespace transmuter::geometry
{
std::vector<Eigen::Vector3d> GeometryIf::sampleMaterial(double spacing) const
{
    if (!std::isfinite(spacing) || spacing <= 0.0)
    {
        ERROR("Material sampling requires a positive finite spacing.");
    }
    const auto box = bounds();
    std::vector<Eigen::Vector3d> points;
    if (!box.isValid) return points;
    const Eigen::Vector3i steps = ((box.max - box.min) / spacing).array().ceil().cast<int>();
    for (int x = 0; x <= steps.x(); ++x)
        for (int y = 0; y <= steps.y(); ++y)
            for (int z = 0; z <= steps.z(); ++z)
            {
                const Eigen::Vector3d point = (box.min + spacing * Eigen::Vector3d(x, y, z)).cwiseMin(box.max);
                if (isInside(point)) points.push_back(point);
            }
    return points;
}

}  // namespace transmuter::geometry
