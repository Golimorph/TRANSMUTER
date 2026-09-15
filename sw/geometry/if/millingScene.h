#ifndef TRANSMUTER_GEOMETRY_MILLING_SCENE_H
#define TRANSMUTER_GEOMETRY_MILLING_SCENE_H

#include <memory>

#include "geometryIf.h"
#include "toolPathIf.h"

namespace transmuter::geometry
{

struct MillingScene
{
    // Immutable shape inputs for this scene. A local stock may be an extracted, already-machined
    // section of a larger workpiece.
    std::shared_ptr<const GeometryIf> stock;
    std::shared_ptr<const GeometryIf> model;

    // Shared live state used by execution and visualization.
    std::shared_ptr<GeometryIf> workpiece;
    std::shared_ptr<ToolPathIf> toolPath;

    // The tool geometry is also an immutable input to the operation.
    std::shared_ptr<const GeometryIf> toolModel;
};

}  // namespace transmuter::geometry
#endif  // TRANSMUTER_GEOMETRY_MILLING_SCENE_H
