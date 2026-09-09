#ifndef TRANSMUTER_PATHGEN_PATH_ALGORITHM_IF_H
#define TRANSMUTER_PATHGEN_PATH_ALGORITHM_IF_H

#include <atomic>
#include <memory>

#include "millingScene.h"
#include "pathGeneratorConfig.h"
#include "pathGenerationResult.h"
#include "pointCloud.h"
#include "trace.h"

namespace transmuter::pathgenerator::algorithms
{

class PathAlgorithmIf
{
   public:
    explicit PathAlgorithmIf(const transmuter::geometry::MillingScene& millingScene)
        : m_stock(millingScene.stock),
          m_model(millingScene.model),
          m_toolModel(millingScene.toolModel)
    {
        if (!m_stock)
        {
            ERROR("stock is null ptr");
        }
        if (!m_model)
        {
            ERROR("model is null ptr");
        }
        if (!m_toolModel)
        {
            ERROR("toolModel is null ptr");
        }

        INFO("Path generation algorithm ready.");
    }

    virtual ~PathAlgorithmIf() = default;

    // Algorithm needs to know the settings
    virtual void setConfig(const PathGeneratorConfig& config) = 0;

    // The supplied path is published live; the returned workpiece is published on completion.
    virtual PathGenerationResult generate(
        std::atomic<bool>& stopRequested,
        std::shared_ptr<geometry::PointCloud> liveToolPath) = 0;

   protected:
    const geometry::PointCloud& stock() const
    {
        return *m_stock;
    }

    const geometry::PointCloud& model() const
    {
        return *m_model;
    }

    const geometry::PointCloud& toolModel() const
    {
        return *m_toolModel;
    }

   private:
    // Retain the immutable inputs for the lifetime of the algorithm.
    std::shared_ptr<const geometry::PointCloud> m_stock;
    std::shared_ptr<const geometry::PointCloud> m_model;
    std::shared_ptr<const geometry::PointCloud> m_toolModel;
};

}  // namespace transmuter::pathgenerator::algorithms

#endif  // TRANSMUTER_PATHGEN_PATH_ALGORITHM_IF_H
