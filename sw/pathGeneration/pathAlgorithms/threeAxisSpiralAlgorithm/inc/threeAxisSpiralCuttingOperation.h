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

/** Size of the origin-centered work volume whose cylinder axis is +Z. */
struct CylindricalMillingVolume
{
    float radius = 0.0f;
    float height = 0.0f;
};

/** Owns the state and cutting mechanics for one three-axis spiral generation run. */
class ThreeAxisSpiralCuttingOperation
{
   public:
    struct Inputs
    {
        CylindricalMillingVolume millingVolume;
        std::shared_ptr<geometry::GeometryIf> initialWorkpiece;
        const geometry::GeometryIf& targetModel;
        const geometry::GeometryIf& toolModel;
        std::shared_ptr<geometry::PointCloud> liveToolPath;
    };

    struct Layer
    {
        float toolZ;
    };

    struct Circle
    {
        float radius;
    };

    explicit ThreeAxisSpiralCuttingOperation(Inputs inputs);

    std::vector<Layer> layersFromTopToBottom(double layerSpacing) const;
    std::vector<Circle> concentricCircles(double passSpacing) const;
    void prepareToCut(const Layer& layer);
    void cutCounterClockwiseAvoidingModel(const Circle& circle, double pointSpacing);
    void finishCurrentLayerAtCenter();

    size_t completedLayerCount() const;
    PathGenerationResult takeResult();

   private:
    struct VerticalToolEnvelope
    {
        float radius;
        float minimumLocalZ;
    };

    void setupGeneration(Inputs inputs);
    VerticalToolEnvelope calculateToolEnvelope(const geometry::GeometryIf& tool) const;
    void moveToolToLayerHeightAvoidingModel(const Eigen::Vector2f& toolCenter);
    void retractToolToSafeHeight();
    void moveToolToNextLayerStart();
    geometry::Vertex appendPathPoint(const Eigen::Vector3f& position);

    std::shared_ptr<geometry::GeometryIf> m_workpiece;
    std::shared_ptr<geometry::PointCloud> m_toolPath;
    std::unique_ptr<geometry::MaterialRemovalIf> m_materialRemoval;
    std::unique_ptr<geometry::VerticalToolClearanceIf> m_modelClearance;
    CylindricalMillingVolume m_millingVolume;
    VerticalToolEnvelope m_tool;
    std::vector<geometry::Vertex> m_currentLayerCuttingPoses;
    float m_outerRadius = 0.0f;
    float m_safeTravelZ = 0.0f;
    float m_layerZ = 0.0f;
    size_t m_completedLayerCount = 0;
    size_t m_pointsSincePublicationPause = 0;
};

}  // namespace transmuter::pathgenerator::algorithms

#endif  // TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_CUTTING_OPERATION_H
