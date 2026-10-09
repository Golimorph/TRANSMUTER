#include "pathComposition.h"

#include <algorithm>
#include <array>
#include <vector>

#include "MachineKinematicsIf.h"
#include "toolSweep.h"
#include "trace.h"

namespace transmuter::pathgenerator
{
namespace
{
double clearanceHeight(const geometry::MillingScene& scene)
{
    constexpr double margin = 3.0;
    if (!scene.stock || !scene.model || !scene.toolModel || !scene.stock->bounds().isValid ||
        !scene.model->bounds().isValid || !scene.toolModel->bounds().isValid)
        ERROR("Path composition needs bounded stock, target, and cutter geometry.");
    return std::max(scene.stock->bounds().max.z(), scene.model->bounds().max.z()) + margin -
           scene.toolModel->bounds().min.z();
}

bool appendTransfer(const MachineMove& previousMove, const MachineMove& next, MachinePathIf& combined, const geometry::MillingScene& scene, double geometrySampleSpacing, const std::atomic<bool>& stop)
{
    const auto previous = scene.machineKinematics->displayPose(previousMove.axes).toolInWorkpiece;
    const auto nextPose = scene.machineKinematics->displayPose(next.axes).toolInWorkpiece;
    auto departure = previous;
    departure.pos.z() = std::max(clearanceHeight(scene), departure.pos.z());
    auto arrival = nextPose;
    arrival.pos.z() = departure.pos.z();

    auto trialMachine = scene.machineKinematics->fork(combined);
    std::array<geometry::Vertex, 3> poses{departure, arrival, nextPose};
    std::vector<MachineMove> transfer;
    geometry::Vertex prior = previous;
    const auto bounds = scene.toolModel->bounds();
    const double reach = bounds.min.cwiseAbs().cwiseMax(bounds.max.cwiseAbs()).norm();
    for (size_t index = 0; index < poses.size(); ++index)
    {
        for (const auto& transform : sampleToolSweep(prior, poses[index], geometrySampleSpacing, reach))
            if (scene.stock->intersects(*scene.toolModel, transform) ||
                scene.model->intersects(*scene.toolModel, transform))
            {
                ERROR("Operation transfer intersects material.");
            }
        const auto motionType = index == 2 ? next.motionType : MotionType::RAPID;
        const auto result = trialMachine->propose(poses[index], motionType, stop);
        if (stop.load())
        {
            return false;
        }
        if (result.status != MachinePathStatus::ACCEPTED)
        {
            ERROR("Machine rejected the operation transfer.");
        }
        const auto path = trialMachine->path();
        transfer.push_back(*path->last());
        prior = poses[index];
    }
    combined.append(transfer[0]);
    combined.append(transfer[1]);
    return true;
}
}  // namespace

void appendThreeAxisOperationPath(const MachinePathIf& operation, MachinePathIf& combined, const geometry::MillingScene& transitionScene, double geometrySampleSpacing, const std::atomic<bool>& stop)
{
    std::vector<MachineMove> moves;
    operation.copyTo(moves);
    if (moves.empty())
    {
        return;
    }
    for (const auto& move : moves)
    {
        if (stop.load())
        {
            return;
        }
        if (!transitionScene.machineKinematics->displayPose(move.axes).toolInWorkpiece.normal.isApprox(Eigen::Vector3d::UnitZ(), 1e-6))
        {
            ERROR("Three-axis path composition requires +Z tool poses.");
        }
    }
    const auto lastCombinedMove = combined.last();
    if (lastCombinedMove && !appendTransfer(*lastCombinedMove, moves.front(), combined, transitionScene, geometrySampleSpacing, stop))
    {
        return;
    }
    for (const auto& move : moves)
    {
        if (stop.load())
        {
            return;
        }
        combined.append(move);
    }
}
}  // namespace transmuter::pathgenerator
