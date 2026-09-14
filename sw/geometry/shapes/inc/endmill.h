#ifndef TRANSMUTER_GEOMETRY_SHAPES_ENDMILL_H
#define TRANSMUTER_GEOMETRY_SHAPES_ENDMILL_H

#include <memory>

#include "cylinder.h"

namespace transmuter::geometry::shapes
{

class Endmill
{
   public:
    /**
     * @brief Creates an analytic cylindrical model for a flat-end mill.
     * @param diameter Tool diameter (e.g., 4.0)
     * @param length Visible length of the tool
     */
    static std::shared_ptr<Cylinder> generate(double diameter, double length);
};

}  // namespace transmuter::geometry::shapes

#endif  // TRANSMUTER_GEOMETRY_SHAPES_ENDMILL_H
