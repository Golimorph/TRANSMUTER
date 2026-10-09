#include "toolSweep.h"

#include <algorithm>
#include <cmath>

#include "toolPose.h"
#include "trace.h"

namespace transmuter::pathgenerator
{
std::vector<Eigen::Affine3d> sampleToolSweep(const geometry::Vertex& from, const geometry::Vertex& to, double spacing, double toolReach)
{
    if (!std::isfinite(spacing) || spacing <= 0.0)
    {
        ERROR("Tool sweep spacing must be positive and finite.");
    }
    const double dot = std::clamp(from.normal.normalized().dot(to.normal.normalized()), -1.0, 1.0);
    const double distance = (to.pos - from.pos).norm() + toolReach * std::acos(dot);
    if (!std::isfinite(distance) || distance / spacing > 1000000.0)
    {
        ERROR("Proposed tool sweep exceeds the sampling limit.");
    }
    const size_t count = std::max<size_t>(1, static_cast<size_t>(std::ceil(distance / spacing)));
    std::vector<Eigen::Affine3d> transforms;
    transforms.reserve(count);
    for (size_t i = 1; i <= count; ++i)
    {
        const auto pose = geometry::interpolatePose(from, to, static_cast<double>(i) / count);
        transforms.push_back(geometry::toolTransform(pose));
    }
    return transforms;
}
}  // namespace transmuter::pathgenerator
