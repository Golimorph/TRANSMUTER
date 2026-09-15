#include "../inc/torusGenerator.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>

#include "trace.h"

namespace transmuter::geometry::pointcloudshapes
{

namespace
{

void validateTorus(double majorRadius, double tubeRadius, double density)
{
    if (majorRadius <= tubeRadius || tubeRadius <= 0.0 || density <= 0.0)
    {
        ERROR("Torus major radius must exceed its positive tube radius, and density must be positive.");
    }
}

Eigen::Vector3d torusNormal(double majorAngle, double tubeAngle)
{
    const double radialNormal = std::cos(tubeAngle);
    return Eigen::Vector3d(
        radialNormal * std::cos(majorAngle), radialNormal * std::sin(majorAngle), std::sin(tubeAngle));
}

Eigen::Vector3d surfacePoint(double majorRadius, double tubeRadius, double majorAngle, double tubeAngle)
{
    const double radialDistance = majorRadius + tubeRadius * std::cos(tubeAngle);
    return Eigen::Vector3d(
        radialDistance * std::cos(majorAngle), radialDistance * std::sin(majorAngle), tubeRadius * std::sin(tubeAngle));
}

void addSurfaceSamples(PointCloud& cloud, double majorRadius, double tubeRadius, double density)
{
    const int majorSteps = static_cast<int>(std::max(3.0, std::ceil(2.0 * std::numbers::pi * majorRadius / density)));
    const int tubeSteps = static_cast<int>(std::max(3.0, std::ceil(2.0 * std::numbers::pi * tubeRadius / density)));
    for (int majorIndex = 0; majorIndex < majorSteps; ++majorIndex)
    {
        const double majorAngle = 2.0 * std::numbers::pi * majorIndex / majorSteps;
        for (int tubeIndex = 0; tubeIndex < tubeSteps; ++tubeIndex)
        {
            const double tubeAngle = 2.0 * std::numbers::pi * tubeIndex / tubeSteps;
            cloud.append(Vertex{.pos = surfacePoint(majorRadius, tubeRadius, majorAngle, tubeAngle),
                .normal = torusNormal(majorAngle, tubeAngle)});
        }
    }
}

void addSolidSamples(PointCloud& cloud, double majorRadius, double tubeRadius, double density)
{
    const int majorSteps = static_cast<int>(std::max(3.0, std::ceil(2.0 * std::numbers::pi * majorRadius / density)));
    const int tubeAngleSteps = static_cast<int>(std::max(3.0, std::ceil(2.0 * std::numbers::pi * tubeRadius / density)));
    const int tubeRadiusSteps = static_cast<int>(std::max(1.0, std::ceil(tubeRadius / density)));
    for (int majorIndex = 0; majorIndex < majorSteps; ++majorIndex)
    {
        const double majorAngle = 2.0 * std::numbers::pi * majorIndex / majorSteps;
        for (int tubeAngleIndex = 0; tubeAngleIndex < tubeAngleSteps; ++tubeAngleIndex)
        {
            const double tubeAngle = 2.0 * std::numbers::pi * tubeAngleIndex / tubeAngleSteps;
            for (int tubeRadiusIndex = 0; tubeRadiusIndex <= tubeRadiusSteps; ++tubeRadiusIndex)
            {
                if (tubeRadiusIndex == 0 && tubeAngleIndex > 0)
                {
                    continue;
                }
                const double sampledTubeRadius = tubeRadius * tubeRadiusIndex / tubeRadiusSteps;
                cloud.append(Vertex{.pos = surfacePoint(majorRadius, sampledTubeRadius, majorAngle, tubeAngle),
                    .normal = torusNormal(majorAngle, tubeAngle)});
            }
        }
    }
}

}  // namespace

std::shared_ptr<PointCloud> TorusGenerator::generate(double majorRadius, double tubeRadius, double density)
{
    validateTorus(majorRadius, tubeRadius, density);
    auto cloud = std::make_shared<PointCloud>();
    addSurfaceSamples(*cloud, majorRadius, tubeRadius, density);
    cloud->finalize();
    return cloud;
}

std::shared_ptr<PointCloud> TorusGenerator::generateSolid(double majorRadius, double tubeRadius, double density)
{
    validateTorus(majorRadius, tubeRadius, density);
    auto cloud = std::make_shared<PointCloud>();
    addSolidSamples(*cloud, majorRadius, tubeRadius, density);
    cloud->finalize();
    return cloud;
}

}  // namespace transmuter::geometry::pointcloudshapes
