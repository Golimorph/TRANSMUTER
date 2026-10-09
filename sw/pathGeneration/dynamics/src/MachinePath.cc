#include "MachinePath.h"

namespace transmuter::pathgenerator
{
void MachinePath::clear()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_moves.clear();
}
void MachinePath::append(const MachineMove& move)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_moves.push_back(move);
}
void MachinePath::copyTo(std::vector<MachineMove>& output) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    output = m_moves;
}
size_t MachinePath::size() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_moves.size();
}
MachineMove MachinePath::at(size_t index) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_moves.at(index);
}
std::optional<MachineMove> MachinePath::last() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_moves.empty())
    {
        return std::nullopt;
    }
    return m_moves.back();
}
}  // namespace transmuter::pathgenerator
