#include "../inc/cylinder.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <numbers>

#include "../../pointCloudShapes/inc/cylinderGenerator.h"
#include "trace.h"

namespace transmuter::geometry::analyticshapes
{

namespace
{

double distanceSquaredFromAxis(const Eigen::Vector3d& point, const Eigen::Vector3d& axisPoint, const Eigen::Vector3d& axisDirection, double axisLengthSquared)
{
    const Eigen::Vector3d offset = point - axisPoint;
    const double axialDistance = offset.dot(axisDirection);
    return offset.squaredNorm() - axialDistance * axialDistance / axisLengthSquared;
}

}  // namespace

Cylinder::Cylinder(double radius, double height) : m_radius(radius), m_height(height)
{
    if (radius <= 0.0 || height <= 0.0)
    {
        ERROR("Cylinder radius and height must be greater than zero.");
    }
}

bool Cylinder::isInside(const Eigen::Vector3d& point) const
{
    return point.head<2>().squaredNorm() <= m_radius * m_radius && point.z() >= 0.0 && point.z() <= m_height;
}

GeometryBounds Cylinder::bounds() const
{
    return GeometryBounds{
        .min = Eigen::Vector3d(-m_radius, -m_radius, 0.0),
        .max = Eigen::Vector3d(m_radius, m_radius, m_height),
        .isValid = true,
    };
}

double Cylinder::maximumDistanceFromAxis(const Eigen::Vector3d& axisPoint, const Eigen::Vector3d& axisDirection) const
{
    const double axisLengthSquared = axisDirection.squaredNorm();
    if (axisLengthSquared <= 0.0)
    {
        ERROR("Geometry axis direction must not be zero.");
    }

    double maximumDistanceSquared = 0.0;
    constexpr size_t angleSteps = 360;
    for (const double z : {0.0, m_height})
    {
        for (size_t index = 0; index < angleSteps; ++index)
        {
            const double angle = 2.0 * std::numbers::pi * index / angleSteps;
            const Eigen::Vector3d point(m_radius * std::cos(angle), m_radius * std::sin(angle), z);
            maximumDistanceSquared = std::max(maximumDistanceSquared, distanceSquaredFromAxis(point, axisPoint, axisDirection, axisLengthSquared));
        }
    }
    return std::sqrt(std::max(maximumDistanceSquared, 0.0));
}

std::shared_ptr<GeometryIf> Cylinder::clone() const
{
    return std::make_shared<Cylinder>(m_radius, m_height);
}

void Cylinder::copyVisualizationVertices(std::vector<Vertex>& output) const
{
    const double density = std::max(std::min(m_radius, m_height) / 10.0, 0.5);
    pointcloudshapes::CylinderGenerator::generate(m_radius, m_height, density)->copyTo(output);
    for (auto& vertex : output)
    {
        vertex.pos.z() += m_height * 0.5;
    }
}

}  // namespace transmuter::geometry::analyticshapes
