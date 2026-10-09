#include "BacklashState.h"

#include <cmath>

namespace transmuter::pathgenerator
{
BacklashState BacklashState::fromPath(const MachinePathIf& path, const std::array<bool, 5>& hasBacklash, const AxisValues& takeupDistance)
{
    BacklashState state;
    const auto lastMove = path.last();
    if (!lastMove)
    {
        return state;
    }
    state.m_previous = lastMove->axes;
    for (size_t axis = 0; axis < 5; ++axis)
    {
        if (!hasBacklash[axis])
        {
            continue;
        }
        for (size_t i = path.size() - 1; i > 0; --i)
        {
            const double travel = axisValues(path.at(i).axes)[axis] - axisValues(path.at(i - 1).axes)[axis];
            if (std::abs(travel) <= 1e-9)
            {
                continue;
            }
            const int direction = travel > 0.0 ? 1 : -1;
            if (state.m_pendingDirection[axis] != 0 && state.m_pendingDirection[axis] != direction)
            {
                break;
            }
            state.m_pendingDirection[axis] = direction;
            state.m_takeupTravel[axis] += std::abs(travel);
            if (state.m_takeupTravel[axis] >= takeupDistance[axis])
            {
                state.m_direction[axis] = direction;
                break;
            }
        }
    }
    return state;
}

void BacklashState::observe(const geometry::MachineAxes& pose, const AxisValues& takeupDistance)
{
    if (m_previous)
    {
        for (size_t axis = 0; axis < 5; ++axis)
        {
            const double travel = axisValues(pose)[axis] - axisValues(*m_previous)[axis];
            if (std::abs(travel) <= 1e-9)
            {
                continue;
            }
            const int direction = travel > 0.0 ? 1 : -1;
            if (direction != m_pendingDirection[axis])
            {
                m_pendingDirection[axis] = direction;
                m_takeupTravel[axis] = 0.0;
                m_direction[axis] = 0;
            }
            m_takeupTravel[axis] += std::abs(travel);
            if (m_takeupTravel[axis] >= takeupDistance[axis])
            {
                m_direction[axis] = direction;
            }
        }
    }
    m_previous = pose;
}
}  // namespace transmuter::pathgenerator
