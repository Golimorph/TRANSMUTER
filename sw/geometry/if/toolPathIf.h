#ifndef TRANSMUTER_GEOMETRY_TOOL_PATH_IF_H
#define TRANSMUTER_GEOMETRY_TOOL_PATH_IF_H

#include <vector>

#include "vertex.h"

namespace transmuter::geometry
{

/** Requested speed class for a tool-path segment; machine settings provide concrete speeds. */
enum class MotionSpeed
{
    CUTTING,
    RAPID
};

struct ToolPathPoint
{
    Vertex pose;

    /**
     * Speed for the movement from the preceding tool-path point to this pose.
     * The first point has no incoming movement, so its incomingSpeed is ignored.
     */
    MotionSpeed incomingSpeed = MotionSpeed::CUTTING;
};

/** Representation-independent ordered sequence of tool poses. */
class ToolPathIf
{
   public:
    virtual ~ToolPathIf() = default;

    virtual void clear() = 0;
    virtual void append(
        const Vertex& pose, MotionSpeed incomingSpeed = MotionSpeed::CUTTING) = 0;
    virtual void copyTo(std::vector<ToolPathPoint>& output) const = 0;
    virtual bool isEmpty() const = 0;
};

}  // namespace transmuter::geometry

#endif  // TRANSMUTER_GEOMETRY_TOOL_PATH_IF_H
