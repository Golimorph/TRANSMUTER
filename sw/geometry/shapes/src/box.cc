#include "box.h"

#include <algorithm>
#include <memory>  // Required for std::shared_ptr

namespace transmuter::geometry::shapes
{

namespace
{

/**
 * @brief Updated helper to accept PointCloud by pointer/reference
 */
void addFace(PointCloud& cloud, const Eigen::Vector3f& center, const Eigen::Vector3f& uDir,
             float uLen, const Eigen::Vector3f& vDir, float vLen, const Eigen::Vector3f& normal,
             float density)
{
    int uSteps = static_cast<int>(std::max(1.0f, uLen / density));
    int vSteps = static_cast<int>(std::max(1.0f, vLen / density));

    for (int i = 0; i <= uSteps; ++i)
    {
        for (int j = 0; j <= vSteps; ++j)
        {
            float uFrac = (float(i) / uSteps - 0.5f);
            float vFrac = (float(j) / vSteps - 0.5f);

            Vertex vtx;
            vtx.pos = center + (uFrac * uLen * uDir) + (vFrac * vLen * vDir);
            vtx.normal = normal;

            cloud.addVertex(vtx);
        }
    }
}

}  // anonymous namespace

std::shared_ptr<PointCloud> Box::generate(const Eigen::Vector3f& dimensions, float density)
{
    const float w = dimensions.x();
    const float h = dimensions.y();
    const float d = dimensions.z();

    int uStepsW = static_cast<int>(std::max(1.0f, w / density)) + 1;
    int vStepsH = static_cast<int>(std::max(1.0f, h / density)) + 1;
    int uStepsD = static_cast<int>(std::max(1.0f, d / density)) + 1;

    size_t total_points = (uStepsW * vStepsH * 2) +  // Front/Back
                          (uStepsD * vStepsH * 2) +  // Left/Right
                          (uStepsW * uStepsD * 2);   // Top/Bottom

    // 1. Instantiate the PointCloud as a shared_ptr
    auto cloud = std::make_shared<PointCloud>();
    cloud->reserve(total_points);

    // 2. Pass the dereferenced pointer to addFace
    // Front (Z+) / Back (Z-)
    addFace(*cloud, {0, 0, d / 2}, {1, 0, 0}, w, {0, 1, 0}, h, {0, 0, 1}, density);
    addFace(*cloud, {0, 0, -d / 2}, {-1, 0, 0}, w, {0, 1, 0}, h, {0, 0, -1}, density);

    // Right (X+) / Left (X-)
    addFace(*cloud, {w / 2, 0, 0}, {0, 0, -1}, d, {0, 1, 0}, h, {1, 0, 0}, density);
    addFace(*cloud, {-w / 2, 0, 0}, {0, 0, 1}, d, {0, 1, 0}, h, {-1, 0, 0}, density);

    // Top (Y+) / Bottom (Y-)
    addFace(*cloud, {0, h / 2, 0}, {1, 0, 0}, w, {0, 0, 1}, d, {0, 1, 0}, density);
    addFace(*cloud, {0, -h / 2, 0}, {1, 0, 0}, w, {0, 0, -1}, d, {0, -1, 0}, density);

    cloud->finalize();

    // 3. Return the shared_ptr (this is efficient and avoids the mutex copy error)
    return cloud;
}

}  // namespace transmuter::geometry::shapes