#pragma once

#include <array>
#include <optional>

#include "MachineMechanicsIf.h"

namespace transmuter::pathgenerator
{
/** Searches sparse joint motions on the machine's forward-kinematic manifold. */
class MachineAxisOptimizer
{
   public:
    MachineAxisOptimizer(const MachineMechanicsIf& mechanics, double reach, double motionTolerance);
    std::vector<geometry::MachineAxes> alternatives(const geometry::Vertex& target, const geometry::MachineAxes& previous) const;

   private:
    struct Score
    {
        size_t movingAxes = 0;
        std::array<double, 5> axisTravel{};
        double travel = 0.0;
        double concentration() const
        {
            double total = 0.0;
            for (double distance : axisTravel) total += distance * distance;
            return total;
        }
    };
    Score score(const geometry::MachineAxes& axes, const geometry::MachineAxes& previous) const;
    std::optional<geometry::MachineAxes> fitWithMovingAxes(const geometry::Vertex& target, const geometry::MachineAxes& previous, unsigned mask) const;
    Eigen::Matrix<double, 5, 1> fitStep(const geometry::MachineAxes& axes, const Eigen::Matrix<double, 6, 1>& error, unsigned mask) const;
    void searchSparseAlternatives(std::vector<geometry::MachineAxes>& result, const geometry::Vertex& target, const geometry::MachineAxes& previous) const;
    void rankAlternatives(std::vector<geometry::MachineAxes>& result, const geometry::MachineAxes& previous) const;
    Eigen::Matrix<double, 6, 1> poseVector(const geometry::MachineAxes& axes) const;
    static geometry::MachineAxes changedAxis(const geometry::MachineAxes& axes, size_t index, double value);
    static bool sameAxes(const geometry::MachineAxes& left, const geometry::MachineAxes& right);

    const MachineMechanicsIf& m_mechanics;
    double m_reach;
    double m_motionTolerance;
};
}  // namespace transmuter::pathgenerator
