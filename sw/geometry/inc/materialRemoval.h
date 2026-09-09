#ifndef TRANSMUTER_GEOMETRY_MATERIAL_REMOVAL_H
#define TRANSMUTER_GEOMETRY_MATERIAL_REMOVAL_H

#include <cstddef>
#include <vector>

#include "pointCloud.h"

namespace transmuter::geometry
{

/**
 * Applies tool poses to a mutable workpiece point cloud.
 *
 * Tool models use local +Z as their axis. Tool-path normals rotate that axis into world space,
 * which keeps this interface usable for both three-axis and future five-axis paths.
 */
class MaterialRemoval
{
   public:
    explicit MaterialRemoval(const PointCloud& toolModel);

    size_t apply(PointCloud& workpiece, const std::vector<Vertex>& toolPoses) const;

   private:
    size_t removeVerticalMaterial(PointCloud& workpiece,
                                  const std::vector<Vertex>& toolPoses) const;
    size_t removeOrientedMaterial(PointCloud& workpiece,
                                  const std::vector<Vertex>& toolPoses) const;

    float m_toolRadius = 0.0f;
    float m_toolMinimumLocalZ = 0.0f;
    float m_toolMaximumLocalZ = 0.0f;
};

}  // namespace transmuter::geometry

#endif  // TRANSMUTER_GEOMETRY_MATERIAL_REMOVAL_H
