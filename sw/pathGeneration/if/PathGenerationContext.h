#ifndef TRANSMUTER_PATH_GENERATION_CONTEXT_H
#define TRANSMUTER_PATH_GENERATION_CONTEXT_H

#include <memory>

#include "MachineKinematicsIf.h"
#include "MachinePathIf.h"

namespace transmuter::pathgenerator
{
/** Program-wide accumulated machine path and immutable machine setup for new operations. */
class PathGenerationContext
{
   public:
    PathGenerationContext(std::shared_ptr<MachinePathIf> machinePath, std::shared_ptr<const MachineKinematicsIf> machineSetup);

    /** Complete path accumulated across accepted operations. */
    const std::shared_ptr<MachinePathIf>& machinePath() const;
    const std::shared_ptr<const MachineKinematicsIf>& machineSetup() const;

   private:
    std::shared_ptr<MachinePathIf> m_machinePath;
    std::shared_ptr<const MachineKinematicsIf> m_machineSetup;
};
}  // namespace transmuter::pathgenerator

#endif  // TRANSMUTER_PATH_GENERATION_CONTEXT_H
