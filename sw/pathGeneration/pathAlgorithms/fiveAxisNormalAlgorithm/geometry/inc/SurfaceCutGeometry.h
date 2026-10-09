#pragma once
#include <optional>
#include <vector>

#include "MachineMove.h"
#include "PathAlgorithmConfig.h"
#include "millingScene.h"

namespace transmuter::pathgenerator::algorithms
{
/** Builds exposed-surface cuts and spherical clearance links; no stock mutation or machine state. */
class SurfaceCutGeometry
{
   public:
    struct Move
    {
        geometry::Vertex pose;
        MotionType motionType;
    };
    SurfaceCutGeometry(const geometry::MillingScene& scene, const PathAlgorithmConfig& config);
    geometry::Vertex initialPose() const;
    std::optional<std::vector<Move>> candidate(const geometry::Vertex& current, const geometry::Vertex& surface, const Eigen::Vector3d& axis, const Eigen::Vector3d& tangent) const;

   private:
    void configureClearance();
    geometry::Vertex clearancePose(const geometry::Vertex& pose) const;
    void appendClearanceLink(std::vector<Move>& path, const geometry::Vertex& current, const geometry::Vertex& destination) const;
    bool cuttingEndpointsClear(const std::vector<Move>& path) const;
    const geometry::GeometryIf& assembly() const;
    const geometry::MillingScene& m_scene;
    const PathAlgorithmConfig& m_config;
    Eigen::Vector3d m_center;
    double m_clearanceRadius;
};
}  // namespace transmuter::pathgenerator::algorithms
