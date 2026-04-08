#ifndef TRANSMUTER_PATHGEN_PATH_ALGORITHM_IF_H
#define TRANSMUTER_PATHGEN_PATH_ALGORITHM_IF_H

#include <memory>

#include "millingScene.h"
#include "pathGeneratorConfig.h"
#include "pointCloud.h"
#include "trace.h"

namespace transmuter::pathgenerator::algorithms
{

class PathAlgorithmIf
{
   public:
    PathAlgorithmIf(const transmuter::geometry::MillingScene& millingScene)
        : p_workPiece(millingScene.workpiece),
          p_toolPath(millingScene.toolPath),
          p_toolModel(millingScene.toolModel)
    {
        if (!p_workPiece)
        {
            ERROR("workpiece is null ptr");
        }
        if (!p_toolPath)
        {
            ERROR("toolPath is null ptr");
        }
        if (!p_toolModel)
        {
            ERROR("toolModel is null ptr");
        }

        INFO("Path generation algorithm ready.");
    }

    virtual ~PathAlgorithmIf() = default;

    // Algorithm needs to know the settings
    virtual void setConfig(const PathGeneratorConfig& config) = 0;

    // The worker method now accepts the stop signal by reference
    virtual void generate(std::atomic<bool>& stopSignal) = 0;

   protected:
    std::shared_ptr<geometry::PointCloud> p_workPiece = nullptr;
    std::shared_ptr<geometry::PointCloud> p_toolPath = nullptr;
    std::shared_ptr<geometry::PointCloud> p_toolModel = nullptr;
};

}  // namespace transmuter::pathgenerator::algorithms

#endif  // TRANSMUTER_PATHGEN_PATH_ALGORITHM_IF_H