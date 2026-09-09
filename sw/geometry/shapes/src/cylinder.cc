#include "cylinder.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "trace.h"

namespace transmuter::geometry::shapes
{

std::shared_ptr<PointCloud> Cylinder::generate(const float radius, const float height,
                                               const float density)
{
    if (radius <= 0.0f || height <= 0.0f || density <= 0.0f)
    {
        ERROR("Cylinder radius, height, and density must be greater than zero.");
    }

    const float PI = M_PI;

    // Calculate number of steps
    int thetaSteps = static_cast<int>(std::max(1.0f, 2.0f * PI * radius / density));
    int zSteps = static_cast<int>(std::max(1.0f, height / density));
    int radialSteps = static_cast<int>(std::max(1.0f, radius / density));

    // Estimate total points
    size_t total_points =
        static_cast<size_t>(thetaSteps) * (zSteps + 1) +          // lateral
        2 * static_cast<size_t>(thetaSteps) * (radialSteps + 1);  // top and bottom

    auto cloud = std::make_shared<PointCloud>();
    cloud->reserve(total_points);

    // Lateral surface
    for (int i = 0; i < thetaSteps; ++i)
    {
        float theta = 2.0f * PI * static_cast<float>(i) / thetaSteps;
        float cosTheta = std::cos(theta);
        float sinTheta = std::sin(theta);

        for (int j = 0; j <= zSteps; ++j)
        {
            float z = -height / 2.0f + height * static_cast<float>(j) / zSteps;

            Vertex vtx;
            vtx.pos = Eigen::Vector3f(radius * cosTheta, radius * sinTheta, z);
            vtx.normal = Eigen::Vector3f(cosTheta, sinTheta, 0.0f);

            cloud->addVertex(vtx);
        }
    }

    // Top face (z = height/2, normal = (0,0,1))
    for (int i = 0; i < thetaSteps; ++i)
    {
        float theta = 2.0f * PI * static_cast<float>(i) / thetaSteps;
        float cosTheta = std::cos(theta);
        float sinTheta = std::sin(theta);

        for (int r = 0; r <= radialSteps; ++r)
        {
            float rad = radius * static_cast<float>(r) / radialSteps;

            Vertex vtx;
            vtx.pos = Eigen::Vector3f(rad * cosTheta, rad * sinTheta, height / 2.0f);
            vtx.normal = Eigen::Vector3f(0.0f, 0.0f, 1.0f);

            cloud->addVertex(vtx);
        }
    }

    // Bottom face (z = -height/2, normal = (0,0,-1))
    for (int i = 0; i < thetaSteps; ++i)
    {
        float theta = 2.0f * PI * static_cast<float>(i) / thetaSteps;
        float cosTheta = std::cos(theta);
        float sinTheta = std::sin(theta);

        for (int r = 0; r <= radialSteps; ++r)
        {
            float rad = radius * static_cast<float>(r) / radialSteps;

            Vertex vtx;
            vtx.pos = Eigen::Vector3f(rad * cosTheta, rad * sinTheta, -height / 2.0f);
            vtx.normal = Eigen::Vector3f(0.0f, 0.0f, -1.0f);

            cloud->addVertex(vtx);
        }
    }

    cloud->finalize();

    return cloud;
}

std::shared_ptr<PointCloud> Cylinder::generateSolid(const float radius, const float height,
                                                    const float density)
{
    if (radius <= 0.0f || height <= 0.0f || density <= 0.0f)
    {
        ERROR("Cylinder radius, height, and density must be greater than zero.");
    }

    const int diameterSteps =
        static_cast<int>(std::max(1.0f, std::ceil(2.0f * radius / density)));
    const int zSteps = static_cast<int>(std::max(1.0f, std::ceil(height / density)));
    const float radiusSquared = radius * radius;
    const float halfHeight = height * 0.5f;

    auto cloud = std::make_shared<PointCloud>();
    const size_t boundingGridSize = static_cast<size_t>(diameterSteps + 1) *
                                    static_cast<size_t>(diameterSteps + 1) *
                                    static_cast<size_t>(zSteps + 1);
    cloud->reserve(boundingGridSize);

    for (int zIndex = 0; zIndex <= zSteps; ++zIndex)
    {
        const float z = -halfHeight + height * static_cast<float>(zIndex) / zSteps;
        for (int xIndex = 0; xIndex <= diameterSteps; ++xIndex)
        {
            const float x = -radius + 2.0f * radius * static_cast<float>(xIndex) / diameterSteps;
            for (int yIndex = 0; yIndex <= diameterSteps; ++yIndex)
            {
                const float y =
                    -radius + 2.0f * radius * static_cast<float>(yIndex) / diameterSteps;
                const float radialSquared = x * x + y * y;
                if (radialSquared > radiusSquared)
                {
                    continue;
                }

                Vertex vertex;
                vertex.pos = Eigen::Vector3f(x, y, z);

                // Interior samples have no unique surface normal. Using the closest cylinder
                // boundary gives stable coloring when a newly cut cross-section is exposed.
                const float radialDistance = std::sqrt(radialSquared);
                const float distanceToSide = radius - radialDistance;
                const float distanceToTop = halfHeight - z;
                const float distanceToBottom = z + halfHeight;
                if (radialDistance > 0.0f && distanceToSide <= distanceToTop &&
                    distanceToSide <= distanceToBottom)
                {
                    vertex.normal = Eigen::Vector3f(x / radialDistance, y / radialDistance, 0.0f);
                }
                else
                {
                    if (distanceToTop <= distanceToBottom)
                    {
                        vertex.normal = Eigen::Vector3f::UnitZ();
                    }
                    else
                    {
                        vertex.normal = -Eigen::Vector3f::UnitZ();
                    }
                }
                cloud->addVertex(vertex);
            }
        }
    }

    cloud->finalize();
    return cloud;
}

}  // namespace transmuter::geometry::shapes
