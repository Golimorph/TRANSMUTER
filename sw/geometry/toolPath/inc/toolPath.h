#ifndef TRANSMUTER_GEOMETRY_TOOL_PATH_H
#define TRANSMUTER_GEOMETRY_TOOL_PATH_H

#include <mutex>
#include <vector>

#include "../../if/toolPathIf.h"

namespace transmuter::geometry
{

/** Thread-safe private storage for an ordered sequence of tool-path points. */
class ToolPath final : public ToolPathIf
{
   public:
    void clear() override;
    void append(const Vertex& pose, MotionSpeed incomingSpeed = MotionSpeed::CUTTING) override;
    void copyTo(std::vector<ToolPathPoint>& output) const override;
    bool isEmpty() const override;

   private:
    std::vector<ToolPathPoint> m_points;
    mutable std::mutex m_mutex;
};

}  // namespace transmuter::geometry

#endif  // TRANSMUTER_GEOMETRY_TOOL_PATH_H
