#include "../inc/endmill.h"

#include <memory>

#include "trace.h"

namespace transmuter::geometry::analyticshapes
{

std::shared_ptr<Cylinder> Endmill::generate(double diameter, double length)
{
    if (diameter <= 0.0 || length <= 0.0)
    {
        ERROR("Endmill diameter and length must be greater than zero.");
    }

    return std::make_shared<Cylinder>(diameter * 0.5, length);
}

}  // namespace transmuter::geometry::analyticshapes
