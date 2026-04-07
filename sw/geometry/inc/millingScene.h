#ifndef TRANSMUTER_GEOMETRY_MILLING_SCENE_H
#define TRANSMUTER_GEOMETRY_MILLING_SCENE_H

#include <memory>

#include "pointCloud.h"

namespace transmuter::geometry
{
struct MillingScene
{
    std::shared_ptr<geometry::PointCloud> workpiece;
    std::shared_ptr<geometry::PointCloud> toolPath;
    std::shared_ptr<geometry::PointCloud> toolModel;
};
}  // namespace transmuter::geometry
#endif  // TRANSMUTER_GEOMETRY_MILLING_SCENE_H