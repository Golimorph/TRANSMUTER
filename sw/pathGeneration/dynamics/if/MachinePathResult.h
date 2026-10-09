#pragma once

namespace transmuter::pathgenerator
{
enum class MachinePathStatus
{
    ACCEPTED,
    UNSAFE,
    UNREACHABLE,
    CANCELLED
};

struct MachinePathResult
{
    MachinePathStatus status = MachinePathStatus::UNREACHABLE;
};
}  // namespace transmuter::pathgenerator
