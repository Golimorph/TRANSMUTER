#include "toolPath.h"

namespace transmuter::geometry
{

void ToolPath::clear()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_points.clear();
}

void ToolPath::append(const Vertex& pose, MotionSpeed incomingSpeed)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_points.push_back(ToolPathPoint{
        .pose = pose,
        .incomingSpeed = incomingSpeed,
    });
}

void ToolPath::copyTo(std::vector<ToolPathPoint>& output) const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    output = m_points;
}

bool ToolPath::isEmpty() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_points.empty();
}

}  // namespace transmuter::geometry
