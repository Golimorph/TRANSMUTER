#ifndef TRANSMUTER_GEOMETRY_VERTICAL_TOOL_CLEARANCE_H
#define TRANSMUTER_GEOMETRY_VERTICAL_TOOL_CLEARANCE_H

#include <Eigen/Geometry>
#include <memory>

#include "geometryIf.h"

namespace transmuter::geometry
{

/** Collision avoidance specialized for a tool whose orientation remains parallel to +Z. */
class VerticalToolClearanceIf
{
   public:
    virtual ~VerticalToolClearanceIf() = default;

    /** Return the requested position, lifted along +Z only when the tool would collide. */
    virtual Eigen::Vector3f liftToAvoidCollision(
        const Eigen::Vector3f& requestedToolPosition, float clearance = 0.0f) const = 0;

    virtual float geometryTolerance() const = 0;
};

std::unique_ptr<VerticalToolClearanceIf> createVerticalToolClearance(
    const GeometryIf& obstacle, const GeometryIf& tool);

}  // namespace transmuter::geometry

#endif  // TRANSMUTER_GEOMETRY_VERTICAL_TOOL_CLEARANCE_H
