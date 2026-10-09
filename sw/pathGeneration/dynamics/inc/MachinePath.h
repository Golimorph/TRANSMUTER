#pragma once

#include <mutex>
#include <optional>

#include "MachinePathIf.h"

namespace transmuter::pathgenerator
{
class MachinePath final : public MachinePathIf
{
   public:
    void clear() override;
    void append(const MachineMove& move) override;
    void copyTo(std::vector<MachineMove>& output) const override;
    size_t size() const override;
    MachineMove at(size_t index) const override;
    std::optional<MachineMove> last() const override;

   private:
    std::vector<MachineMove> m_moves;
    mutable std::mutex m_mutex;
};
}  // namespace transmuter::pathgenerator
