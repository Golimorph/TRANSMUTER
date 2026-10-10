#include "PathGenerationContext.h"

#include <utility>

#include "trace.h"

namespace transmuter::pathgenerator
{
PathGenerationContext::PathGenerationContext(std::shared_ptr<MachinePathIf> machinePath, std::shared_ptr<const MachineKinematicsIf> machineSetup)
    : m_machinePath(std::move(machinePath)), m_machineSetup(std::move(machineSetup))
{
    if (!m_machinePath || !m_machineSetup)
    {
        ERROR("Path generation context requires a machine path and machine setup.");
    }
}

const std::shared_ptr<MachinePathIf>& PathGenerationContext::machinePath() const
{
    return m_machinePath;
}

const std::shared_ptr<const MachineKinematicsIf>& PathGenerationContext::machineSetup() const
{
    return m_machineSetup;
}
}  // namespace transmuter::pathgenerator
