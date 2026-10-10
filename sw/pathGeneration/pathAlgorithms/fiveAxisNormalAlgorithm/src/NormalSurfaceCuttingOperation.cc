#include "NormalSurfaceCuttingOperation.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <sstream>

#include "ExposedSurface.h"
#include "MaterialEngagement.h"
#include "toolSweep.h"
#include "trace.h"

namespace transmuter::pathgenerator::algorithms
{
NormalSurfaceCuttingOperation::NormalSurfaceCuttingOperation(const geometry::MillingScene& scene, const PathAlgorithmConfig& config, geometry::GeometryIf& workpiece, MachineKinematicsIf& machine, const std::atomic<bool>& stop)
    : m_config(config), m_scene(scene), m_workpiece(workpiece), m_machine(machine), m_stop(stop), m_geometry(scene, config)
{
    beginPath();
}
void NormalSurfaceCuttingOperation::beginPath()
{
    const auto initial = m_machine.propose(m_geometry.initialPose(), MotionType::RAPID, m_stop);
    if (initial.status == MachinePathStatus::CANCELLED)
    {
        return;
    }
    if (initial.status != MachinePathStatus::ACCEPTED)
    {
        ERROR("Initial clearance pose cannot be placed on the machine.");
    }
}
bool NormalSurfaceCuttingOperation::peelExposedSurface()
{
    const size_t before = m_accepted;
    const auto surface = ExposedSurface::sample(m_workpiece, m_config.surfaceSampleSpacing, m_config.passSpacing);
    for (const auto& point : surface)
    {
        if (m_stop.load())
        {
            break;
        }
        if (!cutSurfacePoint(point))
        {
            ++m_rejected;
        }
    }
    INFO("Surface pass accepted " << m_accepted - before << " cuts.");
    return m_accepted > before;
}
bool NormalSurfaceCuttingOperation::cutSurfacePoint(const geometry::Vertex& surface)
{
    // Prefer the exposed normal, then allow axial access to cavities and undercuts.
    const std::vector<Eigen::Vector3d> axes{surface.normal, Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY()};
    for (const auto& axis : axes)
    {
        const Eigen::Vector3d firstTangent = axis.unitOrthogonal();
        for (int direction = 0; direction < 8; ++direction)
        {
            if (m_stop.load())
            {
                return false;
            }
            const double angle = direction * std::numbers::pi / 4.0;
            const Eigen::Vector3d tangent = Eigen::AngleAxisd(angle, axis) * firstTangent;
            const auto response = tryDirection(surface, axis, tangent);
            if (response == CandidateResponse::ACCEPTED)
            {
                return true;
            }
            if (response == CandidateResponse::NEXT_ORIENTATION)
            {
                break;
            }
        }
    }
    return false;
}
NormalSurfaceCuttingOperation::CandidateResponse NormalSurfaceCuttingOperation::tryDirection(const geometry::Vertex& surface, const Eigen::Vector3d& axis, const Eigen::Vector3d& tangent)
{
    const auto candidate = m_geometry.candidate(currentPose(), surface, axis, tangent);
    return candidate ? tryCandidate(*candidate) : CandidateResponse::NEXT_DIRECTION;
}
NormalSurfaceCuttingOperation::CandidateResponse NormalSurfaceCuttingOperation::tryCandidate(const std::vector<SurfaceCutGeometry::Move>& candidate)
{
    auto trialMachine = m_machine.fork(*m_machine.path());
    auto trialWorkpiece = m_workpiece.clone();
    std::vector<SurfaceCutGeometry::Move> planned;
    std::vector<std::vector<Eigen::Affine3d>> sweeps;
    bool removedMaterial = false;
    const auto& assembly = m_scene.toolAssemblyModel() ? *m_scene.toolAssemblyModel() : *m_scene.toolModel();
    const auto bounds = assembly.bounds();
    const double reach = bounds.min.cwiseAbs().cwiseMax(bounds.max.cwiseAbs()).norm();
    for (auto point : candidate)
    {
        const auto previous = trialMachine->displayPose(trialMachine->path()->last()->axes).toolInWorkpiece;
        auto sweep = sampleToolSweep(previous, point.pose, m_config.pointSpacing, reach);
        if (!geometryClear(*trialWorkpiece, sweep))
        {
            return CandidateResponse::NEXT_DIRECTION;
        }
        const bool contact = std::any_of(sweep.begin(), sweep.end(), [this, &trialWorkpiece](const auto& transform)
            { return trialWorkpiece->intersects(*m_scene.toolModel(), transform); });
        if (point.motionType == MotionType::RAPID && contact)
        {
            return CandidateResponse::NEXT_DIRECTION;
        }
        point.motionType = contact ? MotionType::CUTTING : MotionType::RAPID;
        const auto result = trialMachine->propose(point.pose, point.motionType, m_stop);
        if (result.status != MachinePathStatus::ACCEPTED)
        {
            return rejectedCandidate(result);
        }
        removedMaterial |= contact;
        planned.push_back(point);
        if (contact)
        {
            trialWorkpiece->cut(*m_scene.toolModel(), sweep);
        }
        sweeps.push_back(std::move(sweep));
    }
    if (!removedMaterial)
    {
        return CandidateResponse::NEXT_DIRECTION;
    }
    // Deterministic planning resolves these same requested poses from the same accepted history.
    return commitCandidate(planned, sweeps) ? CandidateResponse::ACCEPTED : CandidateResponse::NEXT_DIRECTION;
}
NormalSurfaceCuttingOperation::CandidateResponse NormalSurfaceCuttingOperation::rejectedCandidate(const MachinePathResult& result)
{
    if (result.status == MachinePathStatus::CANCELLED)
    {
        return CandidateResponse::NEXT_DIRECTION;
    }
    if (result.status == MachinePathStatus::UNREACHABLE)
    {
        return CandidateResponse::NEXT_ORIENTATION;
    }
    return CandidateResponse::NEXT_DIRECTION;
}
bool NormalSurfaceCuttingOperation::geometryClear(const geometry::GeometryIf& workpiece, const std::vector<Eigen::Affine3d>& sweep) const
{
    const auto& assembly = m_scene.toolAssemblyModel() ? *m_scene.toolAssemblyModel() : *m_scene.toolModel();
    for (const auto& transform : sweep)
    {
        if (m_scene.model()->intersects(assembly, transform))
        {
            return false;
        }
        if (!m_scene.toolAssemblyModel())
        {
            continue;
        }
        const auto inverse = transform.inverse();
        for (const auto& point : MaterialEngagement::sample(workpiece, assembly, transform, m_config.surfaceSampleSpacing))
        {
            if (!m_scene.toolModel()->isInside(inverse * point))
            {
                return false;
            }
        }
    }
    return true;
}
geometry::Vertex NormalSurfaceCuttingOperation::currentPose() const
{
    const auto path = m_machine.path();
    return m_machine.displayPose(path->last()->axes).toolInWorkpiece;
}
bool NormalSurfaceCuttingOperation::commitCandidate(const std::vector<SurfaceCutGeometry::Move>& planned, const std::vector<std::vector<Eigen::Affine3d>>& sweeps)
{
    for (size_t i = 0; i < planned.size(); ++i)
    {
        const auto accepted = m_machine.propose(planned[i].pose, planned[i].motionType, m_stop);
        if (accepted.status == MachinePathStatus::CANCELLED)
        {
            return false;
        }
        if (accepted.status != MachinePathStatus::ACCEPTED)
        {
            ERROR("Machine rejected a previously checked surface move.");
        }
        if (planned[i].motionType == MotionType::CUTTING)
        {
            m_workpiece.cut(*m_scene.toolModel(), sweeps[i]);
        }
    }
    ++m_accepted;
    return true;
}
std::string NormalSurfaceCuttingOperation::generationNote(bool cancelled) const
{
    std::ostringstream note;
    note << "Experimental surface peeling: " << m_accepted << " accepted cuts, " << m_rejected << " unresolved surface candidates. " << (cancelled ? "Cancelled. " : "") << "Coverage is not certified; inaccessible material and sampling residuals may remain.";
    return note.str();
}
}  // namespace transmuter::pathgenerator::algorithms
