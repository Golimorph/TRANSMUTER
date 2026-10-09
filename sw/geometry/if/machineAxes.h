#pragma once
#include <cmath>

namespace transmuter::geometry
{
/** Physical table X/Y, spindle-tip Z in mm; trunnion A/B in continuous radians. */
struct MachineAxes
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double a = 0.0;
    double b = 0.0;
};
inline bool finiteAxes(const MachineAxes& axes)
{
    return std::isfinite(axes.x) && std::isfinite(axes.y) && std::isfinite(axes.z) && std::isfinite(axes.a) && std::isfinite(axes.b);
}
inline MachineAxes interpolateAxes(const MachineAxes& from, const MachineAxes& to, double fraction)
{
    return {from.x + fraction * (to.x - from.x), from.y + fraction * (to.y - from.y), from.z + fraction * (to.z - from.z), from.a + fraction * (to.a - from.a), from.b + fraction * (to.b - from.b)};
}
}  // namespace transmuter::geometry
