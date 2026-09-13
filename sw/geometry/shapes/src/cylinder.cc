#include "cylinder.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <numbers>

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

    const float pi = std::numbers::pi_v<float>;
    const int angleSteps =
        static_cast<int>(std::max(1.0f, 2.0f * pi * radius / density));
    const int heightSteps = static_cast<int>(std::max(1.0f, height / density));
    const int radialSteps = static_cast<int>(std::max(1.0f, radius / density));
    const size_t pointCapacity =
        static_cast<size_t>(angleSteps) * (heightSteps + 1) +
        2 * static_cast<size_t>(angleSteps) * (radialSteps + 1);

    auto cloud = std::make_shared<PointCloud>();
    cloud->reserve(pointCapacity);

    // Lateral surface.
    for (int angleIndex = 0; angleIndex < angleSteps; ++angleIndex)
    {
        const float angle = 2.0f * pi * static_cast<float>(angleIndex) / angleSteps;
        const float cosAngle = std::cos(angle);
        const float sinAngle = std::sin(angle);

        for (int heightIndex = 0; heightIndex <= heightSteps; ++heightIndex)
        {
            const float z = -height * 0.5f +
                            height * static_cast<float>(heightIndex) / heightSteps;
            Vertex vertex;
            vertex.pos = Eigen::Vector3f(radius * cosAngle, radius * sinAngle, z);
            vertex.normal = Eigen::Vector3f(cosAngle, sinAngle, 0.0f);
            cloud->addVertex(vertex);
        }
    }

    // Top face.
    for (int angleIndex = 0; angleIndex < angleSteps; ++angleIndex)
    {
        const float angle = 2.0f * pi * static_cast<float>(angleIndex) / angleSteps;
        const float cosAngle = std::cos(angle);
        const float sinAngle = std::sin(angle);

        for (int radialIndex = 0; radialIndex <= radialSteps; ++radialIndex)
        {
            const float radialDistance =
                radius * static_cast<float>(radialIndex) / radialSteps;
            Vertex vertex;
            vertex.pos = Eigen::Vector3f(radialDistance * cosAngle,
                                         radialDistance * sinAngle, height * 0.5f);
            vertex.normal = Eigen::Vector3f::UnitZ();
            cloud->addVertex(vertex);
        }
    }

    // Bottom face.
    for (int angleIndex = 0; angleIndex < angleSteps; ++angleIndex)
    {
        const float angle = 2.0f * pi * static_cast<float>(angleIndex) / angleSteps;
        const float cosAngle = std::cos(angle);
        const float sinAngle = std::sin(angle);

        for (int radialIndex = 0; radialIndex <= radialSteps; ++radialIndex)
        {
            const float radialDistance =
                radius * static_cast<float>(radialIndex) / radialSteps;
            Vertex vertex;
            vertex.pos = Eigen::Vector3f(radialDistance * cosAngle,
                                         radialDistance * sinAngle, -height * 0.5f);
            vertex.normal = -Eigen::Vector3f::UnitZ();
            cloud->addVertex(vertex);
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
    const int heightSteps =
        static_cast<int>(std::max(1.0f, std::ceil(height / density)));
    const float radiusSquared = radius * radius;
    const float halfHeight = height * 0.5f;
    const size_t pointCapacity = static_cast<size_t>(diameterSteps + 1) *
                                 static_cast<size_t>(diameterSteps + 1) *
                                 static_cast<size_t>(heightSteps + 1);

    auto cloud = std::make_shared<PointCloud>();
    cloud->reserve(pointCapacity);

    for (int heightIndex = 0; heightIndex <= heightSteps; ++heightIndex)
    {
        const float z = -halfHeight +
                        height * static_cast<float>(heightIndex) / heightSteps;
        for (int xIndex = 0; xIndex <= diameterSteps; ++xIndex)
        {
            const float x = -radius +
                            2.0f * radius * static_cast<float>(xIndex) / diameterSteps;
            for (int yIndex = 0; yIndex <= diameterSteps; ++yIndex)
            {
                const float y = -radius +
                                2.0f * radius * static_cast<float>(yIndex) / diameterSteps;
                const float radialSquared = x * x + y * y;
                if (radialSquared > radiusSquared)
                {
                    continue;
                }

                Vertex vertex;
                vertex.pos = Eigen::Vector3f(x, y, z);

                // Choose the closest cylinder boundary for stable exposed-surface coloring.
                const float radialDistance = std::sqrt(radialSquared);
                const float distanceToSide = radius - radialDistance;
                const float distanceToTop = halfHeight - z;
                const float distanceToBottom = z + halfHeight;
                if (radialDistance > 0.0f && distanceToSide <= distanceToTop &&
                    distanceToSide <= distanceToBottom)
                {
                    vertex.normal =
                        Eigen::Vector3f(x / radialDistance, y / radialDistance, 0.0f);
                }
                else if (distanceToTop <= distanceToBottom)
                {
                    vertex.normal = Eigen::Vector3f::UnitZ();
                }
                else
                {
                    vertex.normal = -Eigen::Vector3f::UnitZ();
                }
                cloud->addVertex(vertex);
            }
        }
    }

    cloud->finalize();
    return cloud;
}

}  // namespace transmuter::geometry::shapes
