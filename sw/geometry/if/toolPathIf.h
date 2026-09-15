#ifndef TRANSMUTER_GEOMETRY_TOOL_PATH_IF_H
#define TRANSMUTER_GEOMETRY_TOOL_PATH_IF_H

#include <vector>

#include "vertex.h"

namespace transmuter::geometry
{

/** Representation-independent ordered sequence of tool poses. */
class ToolPathIf
{
   public:
    virtual ~ToolPathIf() = default;

    virtual void clear() = 0;
    virtual void append(const Vertex& pose) = 0;
    virtual void copyTo(std::vector<Vertex>& output) const = 0;
    virtual bool isEmpty() const = 0;
};

}  // namespace transmuter::geometry

#endif  // TRANSMUTER_GEOMETRY_TOOL_PATH_IF_H
