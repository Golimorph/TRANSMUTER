#include "cylinder.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <numbers>

#include "trace.h"

namespace transmuter::geometry::shapes
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
    generate(m_radius, m_height, density)->copyTo(output);
    for (auto& vertex : output)
    {
        vertex.pos.z() += m_height * 0.5;
    }
}

std::shared_ptr<PointCloud> Cylinder::generate(double radius, double height, double density)
{
    if (radius <= 0.0 || height <= 0.0 || density <= 0.0)
    {
        ERROR("Cylinder radius, height, and density must be greater than zero.");
    }

    const double pi = std::numbers::pi;
    const int angleSteps = static_cast<int>(std::max(1.0, 2.0 * pi * radius / density));
    const int heightSteps = static_cast<int>(std::max(1.0, height / density));
    const int radialSteps = static_cast<int>(std::max(1.0, radius / density));
    const size_t pointCapacity = static_cast<size_t>(angleSteps) * (heightSteps + 1) + 2 * static_cast<size_t>(angleSteps) * (radialSteps + 1);

    auto cloud = std::make_shared<PointCloud>();
    cloud->reserve(pointCapacity);

    // Lateral surface.
    for (int angleIndex = 0; angleIndex < angleSteps; ++angleIndex)
    {
        const double angle = 2.0 * pi * angleIndex / angleSteps;
        const double cosAngle = std::cos(angle);
        const double sinAngle = std::sin(angle);

        for (int heightIndex = 0; heightIndex <= heightSteps; ++heightIndex)
        {
            const double z = -height * 0.5 + height * heightIndex / heightSteps;
            Vertex vertex;
            vertex.pos = Eigen::Vector3d(radius * cosAngle, radius * sinAngle, z);
            vertex.normal = Eigen::Vector3d(cosAngle, sinAngle, 0.0);
            cloud->addVertex(vertex);
        }
    }

    // Top face.
    for (int angleIndex = 0; angleIndex < angleSteps; ++angleIndex)
    {
        const double angle = 2.0 * pi * angleIndex / angleSteps;
        const double cosAngle = std::cos(angle);
        const double sinAngle = std::sin(angle);

        for (int radialIndex = 0; radialIndex <= radialSteps; ++radialIndex)
        {
            const double radialDistance = radius * radialIndex / radialSteps;
            Vertex vertex;
            vertex.pos = Eigen::Vector3d(radialDistance * cosAngle, radialDistance * sinAngle, height * 0.5);
            vertex.normal = Eigen::Vector3d::UnitZ();
            cloud->addVertex(vertex);
        }
    }

    // Bottom face.
    for (int angleIndex = 0; angleIndex < angleSteps; ++angleIndex)
    {
        const double angle = 2.0 * pi * angleIndex / angleSteps;
        const double cosAngle = std::cos(angle);
        const double sinAngle = std::sin(angle);

        for (int radialIndex = 0; radialIndex <= radialSteps; ++radialIndex)
        {
            const double radialDistance = radius * radialIndex / radialSteps;
            Vertex vertex;
            vertex.pos = Eigen::Vector3d(radialDistance * cosAngle, radialDistance * sinAngle, -height * 0.5);
            vertex.normal = -Eigen::Vector3d::UnitZ();
            cloud->addVertex(vertex);
        }
    }

    cloud->finalize();
    return cloud;
}

std::shared_ptr<PointCloud> Cylinder::generateSolid(double radius, double height, double density)
{
    if (radius <= 0.0 || height <= 0.0 || density <= 0.0)
    {
        ERROR("Cylinder radius, height, and density must be greater than zero.");
    }

    const int diameterSteps = static_cast<int>(std::max(1.0, std::ceil(2.0 * radius / density)));
    const int heightSteps = static_cast<int>(std::max(1.0, std::ceil(height / density)));
    const double radiusSquared = radius * radius;
    const double halfHeight = height * 0.5;
    const size_t pointCapacity = static_cast<size_t>(diameterSteps + 1) * static_cast<size_t>(diameterSteps + 1) * static_cast<size_t>(heightSteps + 1);

    auto cloud = std::make_shared<PointCloud>();
    cloud->reserve(pointCapacity);

    for (int heightIndex = 0; heightIndex <= heightSteps; ++heightIndex)
    {
        const double z = -halfHeight + height * heightIndex / heightSteps;
        for (int xIndex = 0; xIndex <= diameterSteps; ++xIndex)
        {
            const double x = -radius + 2.0 * radius * xIndex / diameterSteps;
            for (int yIndex = 0; yIndex <= diameterSteps; ++yIndex)
            {
                const double y = -radius + 2.0 * radius * yIndex / diameterSteps;
                const double radialSquared = x * x + y * y;
                if (radialSquared > radiusSquared)
                {
                    continue;
                }

                Vertex vertex;
                vertex.pos = Eigen::Vector3d(x, y, z);

                // Choose the closest cylinder boundary for stable exposed-surface coloring.
                const double radialDistance = std::sqrt(radialSquared);
                const double distanceToSide = radius - radialDistance;
                const double distanceToTop = halfHeight - z;
                const double distanceToBottom = z + halfHeight;
                if (radialDistance > 0.0 && distanceToSide <= distanceToTop && distanceToSide <= distanceToBottom)
                {
                    vertex.normal = Eigen::Vector3d(x / radialDistance, y / radialDistance, 0.0);
                }
                else if (distanceToTop <= distanceToBottom)
                {
                    vertex.normal = Eigen::Vector3d::UnitZ();
                }
                else
                {
                    vertex.normal = -Eigen::Vector3d::UnitZ();
                }
                cloud->addVertex(vertex);
            }
        }
    }

    cloud->finalize();
    return cloud;
}

}  // namespace transmuter::geometry::shapes
