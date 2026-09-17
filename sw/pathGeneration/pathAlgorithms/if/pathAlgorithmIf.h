#ifndef TRANSMUTER_PATHGEN_PATH_ALGORITHM_IF_H
#define TRANSMUTER_PATHGEN_PATH_ALGORITHM_IF_H

#include <atomic>
#include <memory>

#include "geometryIf.h"
#include "millingScene.h"
#include "pathGenerationResult.h"
#include "toolPathIf.h"
#include "trace.h"

namespace transmuter::pathgenerator
{

struct PathGeneratorConfig
{
    double pointSpacing;       // Maximum distance between consecutive tool-path samples.
    double passSpacing;        // Distance between neighbouring passes on the same layer.
    double layerSpacing;       // Distance between neighbouring depth layers.
    double geometryTolerance;  // Smallest meaningful spatial distance in the simulated geometry.
};

}  // namespace transmuter::pathgenerator

namespace transmuter::pathgenerator::algorithms
{

class PathAlgorithmIf
{
   public:
    explicit PathAlgorithmIf(const transmuter::geometry::MillingScene& millingScene)
        : m_stock(millingScene.stock), m_model(millingScene.model), m_toolModel(millingScene.toolModel)
    {
        if (!m_stock)
        {
            ERROR("stock is null");
        }
        if (!m_model)
        {
            ERROR("model is null");
        }
        if (!m_toolModel)
        {
            ERROR("toolModel is null");
        }
        INFO("Path generation algorithm ready.");
    }

    virtual ~PathAlgorithmIf() = default;

    // The supplied path is published live; the returned workpiece is published on completion.
    virtual PathGenerationResult generate(
        std::atomic<bool>& stopRequested, std::shared_ptr<geometry::ToolPathIf> pathOutput) const = 0;

   protected:
    const geometry::GeometryIf& stock() const
    {
        return *m_stock;
    }

    const geometry::GeometryIf& model() const
    {
        return *m_model;
    }

    const geometry::GeometryIf& toolModel() const
    {
        return *m_toolModel;
    }

   private:
    // Retain the immutable inputs for the lifetime of the algorithm.
    std::shared_ptr<const geometry::GeometryIf> m_stock;
    std::shared_ptr<const geometry::GeometryIf> m_model;
    std::shared_ptr<const geometry::GeometryIf> m_toolModel;
};

}  // namespace transmuter::pathgenerator::algorithms

#endif  // TRANSMUTER_PATHGEN_PATH_ALGORITHM_IF_H
