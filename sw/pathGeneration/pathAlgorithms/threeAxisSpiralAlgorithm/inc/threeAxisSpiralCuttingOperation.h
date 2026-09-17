#ifndef TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_CUTTING_OPERATION_H
#define TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_CUTTING_OPERATION_H

#include <cstddef>
#include <memory>
#include <vector>

#include "geometryIf.h"
#include "toolPathIf.h"

namespace transmuter::pathgenerator::algorithms
{

/** Owns the state and cutting mechanics for one three-axis spiral generation run. */
class ThreeAxisSpiralCuttingOperation
{
   public:
    struct Layer
    {
        double toolZ;
    };

    struct Circle
    {
        double radius;
    };

    ThreeAxisSpiralCuttingOperation(const geometry::GeometryIf& stock, const geometry::GeometryIf& model,
        const geometry::GeometryIf& tool, double geometryTolerance,
        geometry::ToolPathIf& pathOutput);

    std::vector<Layer> layers(double layerSpacing) const;
    std::vector<Circle> concentricCircles(double passSpacing) const;
    Eigen::Vector2d center() const;
    void prepareToCut(const Layer& layer);
    void moveToolToLayerHeightAvoidingModel(const Eigen::Vector2d& toolCenter);
    void finishCurrentLayerAtCenter();

    size_t completedLayerCount() const;
    std::shared_ptr<geometry::GeometryIf> takeWorkpiece();

   private:
    struct VerticalToolEnvelope
    {
        double radius;
        double minimumLocalZ;
    };

    void setupGeneration(const geometry::GeometryIf& stock, const geometry::GeometryIf& model,
        const geometry::GeometryIf& tool, double geometryTolerance);
    VerticalToolEnvelope calculateToolEnvelope(const geometry::GeometryIf& tool) const;
    double calculateOuterRadius(const geometry::GeometryIf& stock, double toolRadius) const;
    Eigen::Vector3d liftToAvoidModel(const Eigen::Vector3d& requestedPosition) const;
    bool toolIntersectsModelAt(const Eigen::Vector3d& position) const;
    Eigen::Affine3d toolTransform(const geometry::Vertex& pose) const;
    void cutWorkpieceAtCurrentPoses();
    void retractToolToSafeHeight();
    void moveToolToNextLayerStart();
    geometry::Vertex appendPathPoint(const Eigen::Vector3d& position);

    std::shared_ptr<geometry::GeometryIf> m_workpiece;
    geometry::ToolPathIf& m_pathOutput;
    const geometry::GeometryIf* m_modelGeometry = nullptr;
    const geometry::GeometryIf* m_toolGeometry = nullptr;
    geometry::GeometryBounds m_stockBounds;
    VerticalToolEnvelope m_tool;
    std::vector<geometry::Vertex> m_currentLayerCuttingPoses;
    Eigen::Vector2d m_center = Eigen::Vector2d::Zero();
    double m_outerRadius = 0.0;
    double m_safeToolOriginZ = 0.0;
    double m_layerZ = 0.0;
    double m_geometryTolerance = 0.0;
    size_t m_completedLayerCount = 0;
    size_t m_pointsSincePublicationPause = 0;
};

}  // namespace transmuter::pathgenerator::algorithms

#endif  // TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_CUTTING_OPERATION_H
