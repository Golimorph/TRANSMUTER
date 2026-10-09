#include "MachineAxisOptimizer.h"

#include <Eigen/Dense>
#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <utility>

namespace transmuter::pathgenerator
{
MachineAxisOptimizer::MachineAxisOptimizer(const MachineMechanicsIf& mechanics, double reach, double motionTolerance)
    : m_mechanics(mechanics), m_reach(reach), m_motionTolerance(motionTolerance)
{
}
std::vector<geometry::MachineAxes> MachineAxisOptimizer::alternatives(const geometry::Vertex& target, const geometry::MachineAxes& previous) const
{
    auto result = m_mechanics.candidates(target, previous);
    if (!m_mechanics.hasRedundantMotion())
    {
        return result;
    }
    searchSparseAlternatives(result, target, previous);
    rankAlternatives(result, previous);
    return result;
}
void MachineAxisOptimizer::searchSparseAlternatives(std::vector<geometry::MachineAxes>& result, const geometry::Vertex& target, const geometry::MachineAxes& previous) const
{
    const auto movable = m_mechanics.movableAxes();
    for (int count = 1; count <= 5; ++count)
        for (unsigned mask = 1; mask < 32; ++mask)
        {
            if (std::popcount(mask) != count) continue;
            bool available = true;
            for (size_t axis = 0; axis < 5; ++axis)
                if ((mask & (1u << axis)) && !movable[axis]) available = false;
            if (!available) continue;
            const auto candidate = fitWithMovingAxes(target, previous, mask);
            if (!candidate) continue;
            const auto reachable = m_mechanics.candidates(target, *candidate);
            if (std::none_of(reachable.begin(), reachable.end(),
                    [&candidate](const auto& axes) { return sameAxes(axes, *candidate); }))
                continue;
            if (std::none_of(result.begin(), result.end(),
                    [&candidate](const auto& existing) { return sameAxes(existing, *candidate); }))
                result.push_back(*candidate);
        }
}
void MachineAxisOptimizer::rankAlternatives(std::vector<geometry::MachineAxes>& result, const geometry::MachineAxes& previous) const
{
    std::vector<std::pair<geometry::MachineAxes, Score>> ranked;
    ranked.reserve(result.size());
    double baseline = std::numeric_limits<double>::infinity();
    for (const auto& axes : result)
    {
        auto preference = score(axes, previous);
        baseline = std::min(baseline, preference.travel);
        ranked.emplace_back(axes, std::move(preference));
    }
    const double travelLimit = 4.0 * baseline + m_reach;
    std::erase_if(ranked,
        [travelLimit](const auto& item) { return item.second.travel > travelLimit; });
    std::stable_sort(ranked.begin(), ranked.end(),
        [](const auto& a, const auto& b)
        {
            const auto& x = a.second;
            const auto& y = b.second;
            if (x.movingAxes != y.movingAxes) return x.movingAxes < y.movingAxes;
            if (x.concentration() != y.concentration())
                return x.concentration() > y.concentration();
            return x.travel < y.travel;
        });
    result.clear();
    for (auto& item : ranked) result.push_back(std::move(item.first));
}
std::optional<geometry::MachineAxes> MachineAxisOptimizer::fitWithMovingAxes(const geometry::Vertex& target, const geometry::MachineAxes& previous, unsigned mask) const
{
    geometry::MachineAxes axes = previous;
    Eigen::Matrix<double, 6, 1> desired;
    desired << target.pos, m_reach * target.normal.normalized();
    for (int iteration = 0; iteration < 20; ++iteration)
    {
        const auto actual = poseVector(axes);
        const auto error = desired - actual;
        if (error.head<3>().norm() < 1e-6 && error.tail<3>().norm() < 1e-5) return axes;
        const auto step = fitStep(axes, error, mask);
        if (!step.allFinite()) return std::nullopt;
        for (size_t axis = 0; axis < 5; ++axis)
            if (mask & (1u << axis))
            {
                const double limit = axis < 3 ? 10.0 : 0.2;
                const double next = axisValues(axes)[axis] + std::clamp(step[axis], -limit, limit);
                axes = changedAxis(axes, axis, next);
            }
    }
    return std::nullopt;
}
Eigen::Matrix<double, 5, 1> MachineAxisOptimizer::fitStep(const geometry::MachineAxes& axes, const Eigen::Matrix<double, 6, 1>& error, unsigned mask) const
{
    const auto actual = poseVector(axes);
    Eigen::Matrix<double, 6, 5> jacobian = Eigen::Matrix<double, 6, 5>::Zero();
    for (size_t axis = 0; axis < 5; ++axis)
        if (mask & (1u << axis))
        {
            const double epsilon = axis < 3 ? 1e-4 : 1e-5;
            const auto changed = changedAxis(axes, axis, axisValues(axes)[axis] + epsilon);
            jacobian.col(axis) = (poseVector(changed) - actual) / epsilon;
        }
    const Eigen::Matrix<double, 5, 5> normal = jacobian.transpose() * jacobian + 1e-6 * Eigen::Matrix<double, 5, 5>::Identity();
    return normal.ldlt().solve(jacobian.transpose() * error);
}
Eigen::Matrix<double, 6, 1> MachineAxisOptimizer::poseVector(const geometry::MachineAxes& axes) const
{
    const auto pose = workpieceToolPose(m_mechanics, axes);
    Eigen::Matrix<double, 6, 1> result;
    result << pose.pos, m_reach * pose.normal.normalized();
    return result;
}
MachineAxisOptimizer::Score MachineAxisOptimizer::score(const geometry::MachineAxes& axes, const geometry::MachineAxes& previous) const
{
    Score result;
    const auto prior = workpieceToolPose(m_mechanics, previous);
    for (size_t axis = 0; axis < 5; ++axis)
    {
        if (std::abs(axisValues(axes)[axis] - axisValues(previous)[axis]) <= m_motionTolerance)
            continue;
        ++result.movingAxes;
        const auto partial = workpieceToolPose(m_mechanics, changedAxis(previous, axis, axisValues(axes)[axis]));
        const double distance = (partial.pos - prior.pos).norm() +
                                m_reach * (partial.normal.normalized() - prior.normal.normalized()).norm();
        result.axisTravel[axis] = distance;
        result.travel += distance;
    }
    return result;
}
geometry::MachineAxes MachineAxisOptimizer::changedAxis(const geometry::MachineAxes& axes, size_t index, double value)
{
    auto values = axisValues(axes);
    values[index] = value;
    return {values[0], values[1], values[2], values[3], values[4]};
}
bool MachineAxisOptimizer::sameAxes(const geometry::MachineAxes& left, const geometry::MachineAxes& right)
{
    for (size_t axis = 0; axis < 5; ++axis)
        if (std::abs(axisValues(left)[axis] - axisValues(right)[axis]) > 1e-6) return false;
    return true;
}
}  // namespace transmuter::pathgenerator
