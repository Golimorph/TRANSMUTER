#ifndef TRANSMUTER_GEOMETRY_MILLING_SCENE_H
#define TRANSMUTER_GEOMETRY_MILLING_SCENE_H

#include <memory>

#include "pointCloud.h"

namespace transmuter::geometry
{
struct MillingScene
{
    std::shared_ptr<geometry::PointCloud> stock;  // The initial raw material before milling
    std::shared_ptr<geometry::PointCloud> model;  // The CAD model, the goal of the milling process
    std::shared_ptr<geometry::PointCloud>
        workpiece;  // The current state of the workpiece, updated after each milling step
    std::shared_ptr<geometry::PointCloud> toolPath;   // The complete path that the tool will follow
    std::shared_ptr<geometry::PointCloud> toolModel;  // The geometry of the cutting tool
};
}  // namespace transmuter::geometry
#endif  // TRANSMUTER_GEOMETRY_MILLING_SCENE_H