#pragma once
#include <array>
#include <optional>

#include "MachineMechanicsIf.h"
#include "MachinePathIf.h"

namespace transmuter::pathgenerator
{
/** Feed direction established on each machine axis by accepted moves. */
class BacklashState
{
   public:
    static BacklashState fromPath(const MachinePathIf& path, const std::array<bool, 5>& hasBacklash, const AxisValues& takeupDistance);
    void observe(const geometry::MachineAxes& pose, const AxisValues& takeupDistance);
    int direction(size_t axis) const
    {
        return m_direction[axis];
    }

   private:
    std::optional<geometry::MachineAxes> m_previous;
    std::array<int, 5> m_direction{};
    std::array<int, 5> m_pendingDirection{};
    AxisValues m_takeupTravel{};
};
}  // namespace transmuter::pathgenerator
