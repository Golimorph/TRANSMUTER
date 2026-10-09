#pragma once
#include <Eigen/Geometry>
#include <cstdint>
#include <memory>
#include <vector>
namespace transmuter::visualization
{
struct MachineMeshVertex
{
    Eigen::Vector3f position;
    Eigen::Vector3f normal;
};
struct MachineMesh
{
    std::vector<MachineMeshVertex> vertices;
    std::vector<uint32_t> triangleIndices;
};
struct MachineDrawingPart
{
    std::shared_ptr<const MachineMesh> mesh;
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    Eigen::Vector3f color{0.4f, 0.45f, 0.5f};
};
/** Cached indexed triangle meshes plus instance transforms, independent of any graphics API. */
using MachineDrawing = std::vector<MachineDrawingPart>;
struct MachineDrawingConfig
{
    double workpieceBaseZ = -100.0;
    double toolLength = 80.0;
    Eigen::Vector3d jointPositionInTable{0.0, 0.0, 100.0};
};
}  // namespace transmuter::visualization
