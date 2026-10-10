#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "MachineMove.h"

namespace transmuter::pathgenerator
{
/** Ordered machine-axis moves; tool poses are derived through machine kinematics. */
class MachinePathIf
{
   public:
    virtual ~MachinePathIf() = default;
    virtual void clear() = 0;
    virtual void append(const MachineMove& move) = 0;
    virtual void copyTo(std::vector<MachineMove>& output) const = 0;
    virtual size_t size() const = 0;
    virtual MachineMove at(size_t index) const = 0;
    /** Final recorded move, or empty when the path has no moves. */
    virtual std::optional<MachineMove> last() const = 0;

    /** Append a snapshot of another path's ordered machine moves. */
    MachinePathIf& operator+=(const MachinePathIf& addition)
    {
        std::vector<MachineMove> moves;
        addition.copyTo(moves);
        for (const auto& move : moves)
        {
            append(move);
        }
        return *this;
    }
};
}  // namespace transmuter::pathgenerator
