#ifndef TRANSMUTER_GEOMETRY_MILLING_SCENE_H
#define TRANSMUTER_GEOMETRY_MILLING_SCENE_H

#include <memory>

#include "pointCloud.h"

namespace transmuter::geometry
{
struct MillingScene
{
    // Immutable shape inputs for this scene. A local stock may be an extracted, already-machined
    // section of a larger workpiece.
    std::shared_ptr<const geometry::PointCloud> stock;
    std::shared_ptr<const geometry::PointCloud> model;

    // Shared live state used by execution and visualization.
    std::shared_ptr<geometry::PointCloud>
        workpiece;  // The current state of the workpiece, updated after each milling step
    std::shared_ptr<geometry::PointCloud> toolPath;   // The complete path that the tool will follow

    // The tool geometry is also an immutable input to the operation.
    std::shared_ptr<const geometry::PointCloud> toolModel;
};
}  // namespace transmuter::geometry
#endif  // TRANSMUTER_GEOMETRY_MILLING_SCENE_H
