#ifndef TRANSMUTER_PATH_GENERATOR_PATH_GENERATION_RESULT_H
#define TRANSMUTER_PATH_GENERATOR_PATH_GENERATION_RESULT_H

#include <memory>

#include "pointCloud.h"

namespace transmuter::pathgenerator
{

/**
 * The artifacts produced by one local machining operation.
 *
 * toolPath may reference a scene-owned point cloud that was updated live during generation.
 * workpiece stays private during generation. Both artifacts are read-only once published here.
 */
struct PathGenerationResult
{
    std::shared_ptr<const geometry::PointCloud> toolPath;
    std::shared_ptr<const geometry::PointCloud> workpiece;
};

}  // namespace transmuter::pathgenerator

#endif  // TRANSMUTER_PATH_GENERATOR_PATH_GENERATION_RESULT_H
