#ifndef TRANSMUTER_GEOMETRY_MATERIAL_REMOVAL_H
#define TRANSMUTER_GEOMETRY_MATERIAL_REMOVAL_H

#include <memory>
#include <vector>

#include "geometryIf.h"
#include "vertex.h"

namespace transmuter::geometry
{

/**
 * Representation-independent material subtraction along a sequence of tool poses.
 */
class MaterialRemovalIf
{
   public:
    virtual ~MaterialRemovalIf() = default;

    virtual void apply(GeometryIf& workpiece,
                       const std::vector<Vertex>& toolPoses) const = 0;
};

/** Creates material removal compatible with the workpiece and tool representations. */
std::unique_ptr<MaterialRemovalIf> createMaterialRemoval(const GeometryIf& workpiece,
                                                         const GeometryIf& toolModel);

}  // namespace transmuter::geometry

#endif  // TRANSMUTER_GEOMETRY_MATERIAL_REMOVAL_H
