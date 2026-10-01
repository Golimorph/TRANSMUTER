#include "cylinderGenerator.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>

#include "trace.h"

namespace transmuter::geometry::pointcloudshapes
{

namespace
{

struct SurfaceResolution
{
    int angleSteps;
    int heightSteps;
    int radialSteps;
};

void validateCylinder(double radius, double height, double density)
{
    if (radius <= 0.0 || height <= 0.0 || density <= 0.0)
    {
        ERROR("Cylinder radius, height, and density must be greater than zero.");
    }
}

SurfaceResolution surfaceResolution(double radius, double height, double density)
{
    return SurfaceResolution{
        .angleSteps = static_cast<int>(std::max(3.0, 2.0 * std::numbers::pi * radius / density)),
        .heightSteps = static_cast<int>(std::max(1.0, height / density)),
        .radialSteps = static_cast<int>(std::max(1.0, radius / density)),
    };
}

void addLateralSurface(PointCloud& cloud, double radius, double height, const SurfaceResolution& resolution)
{
    for (int angleIndex = 0; angleIndex < resolution.angleSteps; ++angleIndex)
    {
        const double angle = 2.0 * std::numbers::pi * angleIndex / resolution.angleSteps;
        const Eigen::Vector3d normal(std::cos(angle), std::sin(angle), 0.0);
        for (int heightIndex = 0; heightIndex <= resolution.heightSteps; ++heightIndex)
        {
            const double z = -height * 0.5 + height * heightIndex / resolution.heightSteps;
            cloud.append(Vertex{.pos = Eigen::Vector3d(radius * normal.x(), radius * normal.y(), z),
                .normal = normal});
        }
    }
}

void addEndFace(PointCloud& cloud, double radius, double z, const Eigen::Vector3d& normal, const SurfaceResolution& resolution)
{
    for (int angleIndex = 0; angleIndex < resolution.angleSteps; ++angleIndex)
    {
        const double angle = 2.0 * std::numbers::pi * angleIndex / resolution.angleSteps;
        for (int radialIndex = 0; radialIndex <= resolution.radialSteps; ++radialIndex)
        {
            const double radialDistance = radius * radialIndex / resolution.radialSteps;
            cloud.append(Vertex{
                .pos = Eigen::Vector3d(radialDistance * std::cos(angle), radialDistance * std::sin(angle), z),
                .normal = normal,
            });
        }
    }
}

Eigen::Vector3d closestSurfaceNormal(double x, double y, double z, double radius, double halfHeight)
{
    const double radialDistance = std::hypot(x, y);
    const double distanceToSide = radius - radialDistance;
    const double distanceToTop = halfHeight - z;
    const double distanceToBottom = z + halfHeight;
    if (radialDistance > 0.0 && distanceToSide <= distanceToTop && distanceToSide <= distanceToBottom)
    {
        return Eigen::Vector3d(x / radialDistance, y / radialDistance, 0.0);
    }
    if (distanceToTop <= distanceToBottom)
    {
        return Eigen::Vector3d::UnitZ();
    }
    return -Eigen::Vector3d::UnitZ();
}

void addSolidSamples(PointCloud& cloud, double radius, double height, int diameterSteps, int heightSteps)
{
    const double halfHeight = height * 0.5;
    for (int zIndex = 0; zIndex <= heightSteps; ++zIndex)
    {
        const double z = -halfHeight + height * zIndex / heightSteps;
        for (int xIndex = 0; xIndex <= diameterSteps; ++xIndex)
        {
            const double x = -radius + 2.0 * radius * xIndex / diameterSteps;
            for (int yIndex = 0; yIndex <= diameterSteps; ++yIndex)
            {
                const double y = -radius + 2.0 * radius * yIndex / diameterSteps;
                if (x * x + y * y <= radius * radius)
                {
                    cloud.append(Vertex{.pos = Eigen::Vector3d(x, y, z),
                        .normal = closestSurfaceNormal(x, y, z, radius, halfHeight)});
                }
            }
        }
    }
}

}  // namespace

std::shared_ptr<PointCloud> CylinderGenerator::generate(double radius, double height, double density)
{
    validateCylinder(radius, height, density);
    const SurfaceResolution resolution = surfaceResolution(radius, height, density);
    const size_t capacity = static_cast<size_t>(resolution.angleSteps) * (resolution.heightSteps + 1) +
                            2 * static_cast<size_t>(resolution.angleSteps) * (resolution.radialSteps + 1);
    auto cloud = std::make_shared<PointCloud>(capacity);
    addLateralSurface(*cloud, radius, height, resolution);
    addEndFace(*cloud, radius, height * 0.5, Eigen::Vector3d::UnitZ(), resolution);
    addEndFace(*cloud, radius, -height * 0.5, -Eigen::Vector3d::UnitZ(), resolution);
    cloud->finalize();
    return cloud;
}

std::shared_ptr<PointCloud> CylinderGenerator::generateSolid(double radius, double height, double density)
{
    validateCylinder(radius, height, density);
    const int diameterSteps = static_cast<int>(std::max(2.0, std::ceil(2.0 * radius / density)));
    const int heightSteps = static_cast<int>(std::max(1.0, std::ceil(height / density)));
    const size_t capacity = static_cast<size_t>(diameterSteps + 1) * static_cast<size_t>(diameterSteps + 1) *
                            static_cast<size_t>(heightSteps + 1);
    auto cloud = std::make_shared<PointCloud>(capacity);
    addSolidSamples(*cloud, radius, height, diameterSteps, heightSteps);
    cloud->finalize();
    return cloud;
}

}  // namespace transmuter::geometry::pointcloudshapes
