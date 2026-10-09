#include "MachineMeshBuilder.h"

#include <cmath>
#include <numbers>
namespace transmuter::visualization
{
void MachineMeshBuilder::addTriangle(MachineMesh& mesh, const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c)
{
    const Eigen::Vector3f normal = (b - a).cross(c - a).normalized();
    for (const auto& point : {a, b, c})
    {
        mesh.triangleIndices.push_back(static_cast<uint32_t>(mesh.vertices.size()));
        mesh.vertices.push_back({point, normal});
    }
}
std::shared_ptr<const MachineMesh> MachineMeshBuilder::box()
{
    auto mesh = std::make_shared<MachineMesh>();
    const Eigen::Vector3f v[] = {{-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}, {0.5f, 0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f}, {-0.5f, -0.5f, 0.5f}, {0.5f, -0.5f, 0.5f}, {0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f}};
    const int faces[][4] = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {3, 7, 6, 2}, {0, 4, 7, 3}, {1, 2, 6, 5}};
    for (const auto& face : faces)
    {
        addTriangle(*mesh, v[face[0]], v[face[1]], v[face[2]]);
        addTriangle(*mesh, v[face[0]], v[face[2]], v[face[3]]);
    }
    return mesh;
}
std::shared_ptr<const MachineMesh> MachineMeshBuilder::cylinder()
{
    auto mesh = std::make_shared<MachineMesh>();
    constexpr int sides = 32;
    for (int i = 0; i < sides; ++i)
    {
        const float a = 2.0f * std::numbers::pi_v<float> * i / sides;
        const float b = 2.0f * std::numbers::pi_v<float> * (i + 1) / sides;
        const Eigen::Vector3f lowA{std::cos(a), std::sin(a), -0.5f}, lowB{std::cos(b), std::sin(b), -0.5f};
        const Eigen::Vector3f highA{std::cos(a), std::sin(a), 0.5f}, highB{std::cos(b), std::sin(b), 0.5f};
        addTriangle(*mesh, lowA, lowB, highB);
        addTriangle(*mesh, lowA, highB, highA);
        addTriangle(*mesh, {0.0f, 0.0f, -0.5f}, lowB, lowA);
        addTriangle(*mesh, {0.0f, 0.0f, 0.5f}, highA, highB);
    }
    return mesh;
}
}  // namespace transmuter::visualization
