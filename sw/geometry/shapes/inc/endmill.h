#ifndef TRANSMUTER_GEOMETRY_SHAPES_TOOL_H
#define TRANSMUTER_GEOMETRY_SHAPES_TOOL_H

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
    static std::shared_ptr<PointCloud> generate(float diameter, float length)
    {
        auto tool = std::make_shared<PointCloud>();
        float radius = diameter / 2.0f;

        // Generate a cylinder of points
        for (float h = 0; h < length; h += 1.0f)
        {
            for (float a = 0; a < 360.0f; a += 15.0f)
            {
                float rad = a * M_PI / 180.0f;
                Vertex v;
                v.pos = Eigen::Vector3f(cos(rad) * radius, sin(rad) * radius, h);
                v.normal = Eigen::Vector3f(cos(rad), sin(rad), 0.0f);  // Sideways normals
                tool->addVertex(v);
            }
        }
        return tool;
    }
};

}  // namespace transmuter::geometry::shapes
#endif