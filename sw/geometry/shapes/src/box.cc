#include "box.h"

#include <algorithm>
#include <cstddef>
#include <memory>

#include "trace.h"

namespace transmuter::geometry::shapes
{

namespace
{

void validateBox(const Eigen::Vector3f& dimensions, float density)
{
    if ((dimensions.array() <= 0.0f).any() || density <= 0.0f)
    {
        ERROR("Box dimensions and density must be greater than zero.");
    }
}

void addFace(PointCloud& cloud, const Eigen::Vector3f& center, const Eigen::Vector3f& uDir,
             float uLen, const Eigen::Vector3f& vDir, float vLen, const Eigen::Vector3f& normal,
             float density)
{
    const int uSteps = static_cast<int>(std::max(1.0f, uLen / density));
    const int vSteps = static_cast<int>(std::max(1.0f, vLen / density));

    for (int i = 0; i <= uSteps; ++i)
    {
        for (int j = 0; j <= vSteps; ++j)
        {
            const float uFraction = static_cast<float>(i) / uSteps - 0.5f;
            const float vFraction = static_cast<float>(j) / vSteps - 0.5f;

            Vertex vertex;
            vertex.pos = center + uFraction * uLen * uDir + vFraction * vLen * vDir;
            vertex.normal = normal;
            cloud.addVertex(vertex);
        }
    }
}

}  // namespace

std::shared_ptr<PointCloud> Box::generate(const Eigen::Vector3f& dimensions, float density)
{
    validateBox(dimensions, density);
    const float width = dimensions.x();
    const float height = dimensions.y();
    const float depth = dimensions.z();

    const size_t widthSteps =
        static_cast<size_t>(std::max(1.0f, width / density)) + 1;
    const size_t heightSteps =
        static_cast<size_t>(std::max(1.0f, height / density)) + 1;
    const size_t depthSteps =
        static_cast<size_t>(std::max(1.0f, depth / density)) + 1;
    const size_t pointCapacity =
        2 * (widthSteps * heightSteps + depthSteps * heightSteps + widthSteps * depthSteps);

    auto cloud = std::make_shared<PointCloud>(pointCapacity);
    addFace(*cloud, {0, 0, depth / 2}, {1, 0, 0}, width, {0, 1, 0}, height,
            {0, 0, 1}, density);
    addFace(*cloud, {0, 0, -depth / 2}, {-1, 0, 0}, width, {0, 1, 0}, height,
            {0, 0, -1}, density);
    addFace(*cloud, {width / 2, 0, 0}, {0, 0, -1}, depth, {0, 1, 0}, height,
            {1, 0, 0}, density);
    addFace(*cloud, {-width / 2, 0, 0}, {0, 0, 1}, depth, {0, 1, 0}, height,
            {-1, 0, 0}, density);
    addFace(*cloud, {0, height / 2, 0}, {1, 0, 0}, width, {0, 0, 1}, depth,
            {0, 1, 0}, density);
    addFace(*cloud, {0, -height / 2, 0}, {1, 0, 0}, width, {0, 0, -1}, depth,
            {0, -1, 0}, density);
    cloud->finalize();
    return cloud;
}

}  // namespace transmuter::geometry::shapes
