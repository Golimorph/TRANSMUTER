#include "endmill.h"

#include <cmath>
#include <memory>
#include <numbers>

#include "trace.h"

namespace transmuter::geometry::shapes
{

std::shared_ptr<PointCloud> Endmill::generate(float diameter, float length)
{
    if (diameter <= 0.0f || length <= 0.0f)
    {
        ERROR("Endmill diameter and length must be greater than zero.");
    }

    auto tool = std::make_shared<PointCloud>();
    const float radius = diameter * 0.5f;
    for (float height = 0.0f; height < length; height += 1.0f)
    {
        for (float angle = 0.0f; angle < 360.0f; angle += 15.0f)
        {
            const float radians = angle * std::numbers::pi_v<float> / 180.0f;
            const Eigen::Vector2f radial(std::cos(radians), std::sin(radians));
            Vertex vertex;
            vertex.pos = Eigen::Vector3f(radial.x() * radius, radial.y() * radius, height);
            vertex.normal = Eigen::Vector3f(radial.x(), radial.y(), 0.0f);
            tool->addVertex(vertex);
        }
    }
    return tool;
}

}  // namespace transmuter::geometry::shapes
