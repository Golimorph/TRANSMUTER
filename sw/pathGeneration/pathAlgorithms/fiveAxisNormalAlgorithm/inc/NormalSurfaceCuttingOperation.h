#pragma once

#include <atomic>
#include <string>
#include <vector>

#include "MachineKinematicsIf.h"
#include "SurfaceCutGeometry.h"

namespace transmuter::pathgenerator::algorithms
{
class NormalSurfaceCuttingOperation
{
   public:
    NormalSurfaceCuttingOperation(const geometry::MillingScene& scene, const PathAlgorithmConfig& config, geometry::GeometryIf& workpiece, MachineKinematicsIf& machine, const std::atomic<bool>& stop);
    bool peelExposedSurface();
    std::string generationNote(bool cancelled) const;

   private:
    enum class CandidateResponse
    {
        ACCEPTED,
        NEXT_DIRECTION,
        NEXT_ORIENTATION
    };

    void beginPath();
    bool cutSurfacePoint(const geometry::Vertex& surface);
    CandidateResponse tryDirection(const geometry::Vertex& surface, const Eigen::Vector3d& axis, const Eigen::Vector3d& tangent);
    CandidateResponse tryCandidate(const std::vector<SurfaceCutGeometry::Move>& candidate);
    static CandidateResponse rejectedCandidate(const MachinePathResult& result);
    bool geometryClear(const geometry::GeometryIf& workpiece, const std::vector<Eigen::Affine3d>& sweep) const;
    geometry::Vertex currentPose() const;
    bool commitCandidate(const std::vector<SurfaceCutGeometry::Move>& planned, const std::vector<std::vector<Eigen::Affine3d>>& sweeps);
    const PathAlgorithmConfig& m_config;
    const geometry::MillingScene& m_scene;
    geometry::GeometryIf& m_workpiece;
    MachineKinematicsIf& m_machine;
    const std::atomic<bool>& m_stop;
    SurfaceCutGeometry m_geometry;
    size_t m_accepted = 0;
    size_t m_rejected = 0;
};
}  // namespace transmuter::pathgenerator::algorithms
