#include "pointCloud.h"

#include <algorithm>
#include <cmath>

#include "trace.h"

namespace transmuter::geometry
{

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

float PointCloud::maximumDistanceFromAxis(const Eigen::Vector3f& axisPoint,
                                          const Eigen::Vector3f& axisDirection) const
{
    const float axisLengthSquared = axisDirection.squaredNorm();
    if (axisLengthSquared <= 0.0f)
    {
        ERROR("Geometry axis direction must not be zero.");
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    float maximumDistanceSquared = 0.0f;
    for (const auto& vertex : m_vertices)
    {
        const Eigen::Vector3f offset = vertex.pos - axisPoint;
        const float axialDistance = offset.dot(axisDirection);
        const float axialDistanceSquared =
            axialDistance * axialDistance / axisLengthSquared;
        maximumDistanceSquared =
            std::max(maximumDistanceSquared, offset.squaredNorm() - axialDistanceSquared);
    }
    return std::sqrt(std::max(maximumDistanceSquared, 0.0f));
}

}  // namespace transmuter::geometry
