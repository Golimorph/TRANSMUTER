#pragma once
#include "vertex.h"

namespace transmuter::pathgenerator
{
class ToolForceIf
{
   public:
    virtual ~ToolForceIf() = default;
    /** Force exerted by the cutter on the workpiece, in workpiece coordinates. */
    virtual Eigen::Vector3d forceOnWorkpiece(const geometry::Vertex& from, const geometry::Vertex& to) const = 0;
};
}  // namespace transmuter::pathgenerator
