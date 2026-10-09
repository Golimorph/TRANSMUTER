#ifndef TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_CUTTING_OPERATION_H
#define TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_CUTTING_OPERATION_H

#include <atomic>
#include <cstddef>
#include <vector>

#include "MachineKinematicsIf.h"
#include "geometryIf.h"

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

    ThreeAxisSpiralCuttingOperation(const geometry::GeometryIf& stock, const geometry::GeometryIf& model, const geometry::GeometryIf& tool, double geometryTolerance, geometry::GeometryIf& workpiece, MachineKinematicsIf& machine, const std::atomic<bool>& stop);

    std::vector<Layer> layers(double layerSpacing) const;
    std::vector<Circle> concentricCircles(double passSpacing) const;
    Eigen::Vector2d center() const;
    void prepareToCut(const Layer& layer);
    void moveToolToLayerHeightAvoidingModel(const Eigen::Vector2d& toolCenter);
    void finishCurrentLayerAtCenter();

    size_t completedLayerCount() const;

   private:
    struct VerticalToolEnvelope
    {
        double radius;
        double minimumLocalZ;
    };

    void setupGeneration(const geometry::GeometryIf& stock, double geometryTolerance);
    static VerticalToolEnvelope calculateToolEnvelope(const geometry::GeometryIf& tool);
    double calculateOuterRadius(const geometry::GeometryIf& stock, double toolRadius) const;
    geometry::Vertex liftToAvoidModel(const geometry::Vertex& requestedPose) const;
    bool toolIntersectsModelAt(const geometry::Vertex& pose) const;
    static geometry::Vertex verticalToolPose(double x, double y, double z);
    void moveToolTo(const geometry::Vertex& pose, MotionType motionType);
    void retractToolToSafeHeight();
    void moveToolToNextLayerStart();

    const geometry::GeometryIf& m_modelGeometry;
    const geometry::GeometryIf& m_toolGeometry;
    geometry::GeometryIf& m_workpiece;
    MachineKinematicsIf& m_machine;
    const std::atomic<bool>& m_stop;
    geometry::GeometryBounds m_stockBounds;
    VerticalToolEnvelope m_tool;
    Eigen::Vector2d m_center = Eigen::Vector2d::Zero();
    double m_outerRadius = 0.0;
    double m_safeToolOriginZ = 0.0;
    double m_layerZ = 0.0;
    double m_geometryTolerance = 0.0;
    size_t m_completedLayerCount = 0;
};

}  // namespace transmuter::pathgenerator::algorithms

#endif  // TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_CUTTING_OPERATION_H
