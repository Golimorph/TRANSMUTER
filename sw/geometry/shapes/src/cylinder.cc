#include "cylinder.h"

#include <algorithm>
#include <cmath>
#include <memory>

namespace transmuter::geometry::shapes
{

std::shared_ptr<PointCloud> Cylinder::generate(const float radius, const float height,
                                               const float density)
{
    const float PI = M_PI;

    // Calculate number of steps
    int thetaSteps = static_cast<int>(std::max(1.0f, 2.0f * PI * radius / density));
    int ySteps = static_cast<int>(std::max(1.0f, height / density));
    int radialSteps = static_cast<int>(std::max(1.0f, radius / density));

    // Estimate total points
    size_t total_points =
        static_cast<size_t>(thetaSteps) * (ySteps + 1) +          // lateral
        2 * static_cast<size_t>(thetaSteps) * (radialSteps + 1);  // top and bottom

    auto cloud = std::make_shared<PointCloud>();
    cloud->reserve(total_points);

    // Lateral surface
    for (int i = 0; i < thetaSteps; ++i)
    {
        float theta = 2.0f * PI * static_cast<float>(i) / thetaSteps;
        float cosTheta = std::cos(theta);
        float sinTheta = std::sin(theta);

        for (int j = 0; j <= ySteps; ++j)
        {
            float y = -height / 2.0f + height * static_cast<float>(j) / ySteps;

            Vertex vtx;
            vtx.pos = Eigen::Vector3f(radius * cosTheta, y, radius * sinTheta);
            vtx.normal = Eigen::Vector3f(cosTheta, 0.0f, sinTheta);

            cloud->addVertex(vtx);
        }
    }

    // Top face (y = height/2, normal = (0,1,0))
    for (int i = 0; i < thetaSteps; ++i)
    {
        float theta = 2.0f * PI * static_cast<float>(i) / thetaSteps;
        float cosTheta = std::cos(theta);
        float sinTheta = std::sin(theta);

        for (int r = 0; r <= radialSteps; ++r)
        {
            float rad = radius * static_cast<float>(r) / radialSteps;

            Vertex vtx;
            vtx.pos = Eigen::Vector3f(rad * cosTheta, height / 2.0f, rad * sinTheta);
            vtx.normal = Eigen::Vector3f(0.0f, 1.0f, 0.0f);

            cloud->addVertex(vtx);
        }
    }

    // Bottom face (y = -height/2, normal = (0,-1,0))
    for (int i = 0; i < thetaSteps; ++i)
    {
        float theta = 2.0f * PI * static_cast<float>(i) / thetaSteps;
        float cosTheta = std::cos(theta);
        float sinTheta = std::sin(theta);

        for (int r = 0; r <= radialSteps; ++r)
        {
            float rad = radius * static_cast<float>(r) / radialSteps;

            Vertex vtx;
            vtx.pos = Eigen::Vector3f(rad * cosTheta, -height / 2.0f, rad * sinTheta);
            vtx.normal = Eigen::Vector3f(0.0f, -1.0f, 0.0f);

            cloud->addVertex(vtx);
        }
    }

    cloud->finalize();

    return cloud;
}

}  // namespace transmuter::geometry::shapes