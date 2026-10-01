#include "boxGenerator.h"

#include <algorithm>
#include <cstddef>
#include <memory>

#include "trace.h"

namespace transmuter::geometry::pointcloudshapes
{

namespace
{

void validateBox(const Eigen::Vector3d& dimensions, double density)
{
    if ((dimensions.array() <= 0.0).any() || density <= 0.0)
    {
        ERROR("Box dimensions and density must be greater than zero.");
    }
}

void addFace(PointCloud& cloud, const Eigen::Vector3d& center, const Eigen::Vector3d& uDir, double uLen, const Eigen::Vector3d& vDir, double vLen, const Eigen::Vector3d& normal, double density)
{
    const int uSteps = static_cast<int>(std::max(1.0, uLen / density));
    const int vSteps = static_cast<int>(std::max(1.0, vLen / density));

    for (int i = 0; i <= uSteps; ++i)
    {
        for (int j = 0; j <= vSteps; ++j)
        {
            const double uFraction = static_cast<double>(i) / uSteps - 0.5;
            const double vFraction = static_cast<double>(j) / vSteps - 0.5;

            Vertex vertex;
            vertex.pos = center + uFraction * uLen * uDir + vFraction * vLen * vDir;
            vertex.normal = normal;
            cloud.append(vertex);
        }
    }
}

Eigen::Vector3d nearestFaceNormal(const Eigen::Vector3d& point, const Eigen::Vector3d& halfDimensions)
{
    Eigen::Index closestAxis = 0;
    (halfDimensions - point.cwiseAbs()).minCoeff(&closestAxis);
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    normal[closestAxis] = point[closestAxis] < 0.0 ? -1.0 : 1.0;
    return normal;
}

Eigen::Vector3i solidSampleSteps(const Eigen::Vector3d& dimensions, double density)
{
    return ((dimensions / density).array().ceil().cwiseMax(1.0).cast<int>()).matrix();
}

void addSolidSamples(PointCloud& cloud, const Eigen::Vector3d& dimensions, const Eigen::Vector3i& steps)
{
    const Eigen::Vector3d halfDimensions = dimensions * 0.5;
    for (int x = 0; x <= steps.x(); ++x)
    {
        for (int y = 0; y <= steps.y(); ++y)
        {
            for (int z = 0; z <= steps.z(); ++z)
            {
                const Eigen::Vector3d fraction(static_cast<double>(x) / steps.x(),
                    static_cast<double>(y) / steps.y(), static_cast<double>(z) / steps.z());
                Vertex vertex;
                vertex.pos = -halfDimensions + dimensions.cwiseProduct(fraction);
                vertex.normal = nearestFaceNormal(vertex.pos, halfDimensions);
                cloud.append(vertex);
            }
        }
    }
}

}  // namespace

std::shared_ptr<PointCloud> BoxGenerator::generate(const Eigen::Vector3d& dimensions, double density)
{
    validateBox(dimensions, density);
    const double width = dimensions.x();
    const double height = dimensions.y();
    const double depth = dimensions.z();

    const size_t widthSteps = static_cast<size_t>(std::max(1.0, width / density)) + 1;
    const size_t heightSteps = static_cast<size_t>(std::max(1.0, height / density)) + 1;
    const size_t depthSteps = static_cast<size_t>(std::max(1.0, depth / density)) + 1;
    const size_t pointCapacity = 2 * (widthSteps * heightSteps + depthSteps * heightSteps + widthSteps * depthSteps);

    auto cloud = std::make_shared<PointCloud>(pointCapacity);
    addFace(*cloud, {0, 0, depth / 2}, {1, 0, 0}, width, {0, 1, 0}, height, {0, 0, 1}, density);
    addFace(*cloud, {0, 0, -depth / 2}, {-1, 0, 0}, width, {0, 1, 0}, height, {0, 0, -1}, density);
    addFace(*cloud, {width / 2, 0, 0}, {0, 0, -1}, depth, {0, 1, 0}, height, {1, 0, 0}, density);
    addFace(*cloud, {-width / 2, 0, 0}, {0, 0, 1}, depth, {0, 1, 0}, height, {-1, 0, 0}, density);
    addFace(*cloud, {0, height / 2, 0}, {1, 0, 0}, width, {0, 0, 1}, depth, {0, 1, 0}, density);
    addFace(*cloud, {0, -height / 2, 0}, {1, 0, 0}, width, {0, 0, -1}, depth, {0, -1, 0}, density);
    cloud->finalize();
    return cloud;
}

std::shared_ptr<PointCloud> BoxGenerator::generateSolid(const Eigen::Vector3d& dimensions, double density)
{
    validateBox(dimensions, density);
    const Eigen::Vector3i steps = solidSampleSteps(dimensions, density);
    const size_t capacity = static_cast<size_t>(steps.x() + 1) *
                            static_cast<size_t>(steps.y() + 1) *
                            static_cast<size_t>(steps.z() + 1);
    auto cloud = std::make_shared<PointCloud>(capacity);
    addSolidSamples(*cloud, dimensions, steps);
    cloud->finalize();
    return cloud;
}

}  // namespace transmuter::geometry::pointcloudshapes
