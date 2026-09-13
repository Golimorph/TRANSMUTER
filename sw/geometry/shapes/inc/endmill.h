#ifndef TRANSMUTER_GEOMETRY_SHAPES_ENDMILL_H
#define TRANSMUTER_GEOMETRY_SHAPES_ENDMILL_H

#include <memory>

#include "pointCloud.h"

namespace transmuter::geometry::shapes
{

class Endmill
{
   public:
    /**
     * @brief Generates a point cloud for a flat-end mill.
     * @param diameter Tool diameter (e.g., 4.0f)
     * @param length Visible length of the tool
     */
    static std::shared_ptr<PointCloud> generate(float diameter, float length);
};

}  // namespace transmuter::geometry::shapes

#endif  // TRANSMUTER_GEOMETRY_SHAPES_ENDMILL_H
