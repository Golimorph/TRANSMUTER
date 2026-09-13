#ifndef TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_CUTTING_OPERATION_H
#define TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_CUTTING_OPERATION_H

#include <cstddef>
#include <memory>
#include <vector>

#include "geometryIf.h"
#include "materialRemoval.h"
#include "pathGenerationResult.h"
#include "pointCloud.h"
#include "verticalToolClearance.h"

namespace transmuter::pathgenerator::algorithms
{

/** Owns the state and cutting mechanics for one three-axis spiral generation run. */
class ThreeAxisSpiralCuttingOperation
{
   public:
    struct Layer
    {
        float toolZ;
    };

    struct Circle
    {
        float radius;
    };

    ThreeAxisSpiralCuttingOperation(
        const geometry::GeometryIf& stock, const geometry::GeometryIf& model,
        const geometry::GeometryIf& tool,
        std::shared_ptr<geometry::PointCloud> liveToolPath);

    std::vector<Layer> layersFromTopToBottom(double layerSpacing) const;
    std::vector<Circle> concentricCircles(double passSpacing) const;
    void prepareToCut(const Layer& layer);
    void cutCounterClockwise(const Circle& circle, double pointSpacing);
    void finishCurrentLayerAtCenter();

    size_t completedLayerCount() const;
    PathGenerationResult takeResult();

   private:
    struct VerticalToolEnvelope
    {
        float radius;
        float minimumLocalZ;
    };

    void setupGeneration(const geometry::GeometryIf& stock,
                         const geometry::GeometryIf& model,
                         const geometry::GeometryIf& tool,
                         std::shared_ptr<geometry::PointCloud> liveToolPath);
    VerticalToolEnvelope calculateToolEnvelope(const geometry::GeometryIf& tool) const;
    float calculateOuterRadius(const geometry::GeometryIf& stock, float toolRadius) const;
    void moveToolToLayerHeightAvoidingModel(const Eigen::Vector2f& toolCenter);
    void retractToolToSafeHeight();
    void moveToolToNextLayerStart();
    geometry::Vertex appendPathPoint(const Eigen::Vector3f& position);

    std::shared_ptr<geometry::GeometryIf> m_workpiece;
    std::shared_ptr<geometry::PointCloud> m_toolPath;
    std::unique_ptr<geometry::MaterialRemovalIf> m_materialRemoval;
    std::unique_ptr<geometry::VerticalToolClearanceIf> m_modelClearance;
    geometry::GeometryBounds m_stockBounds;
    VerticalToolEnvelope m_tool;
    std::vector<geometry::Vertex> m_currentLayerCuttingPoses;
    Eigen::Vector2f m_center = Eigen::Vector2f::Zero();
    float m_outerRadius = 0.0f;
    float m_safeTravelZ = 0.0f;
    float m_layerZ = 0.0f;
    size_t m_completedLayerCount = 0;
    size_t m_pointsSincePublicationPause = 0;
};

}  // namespace transmuter::pathgenerator::algorithms

#endif  // TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_CUTTING_OPERATION_H
