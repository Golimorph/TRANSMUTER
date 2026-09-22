#ifndef TRANSMUTER_PATHGEN_THREE_AXIS_RASTER_CUTTING_OPERATION_H
#define TRANSMUTER_PATHGEN_THREE_AXIS_RASTER_CUTTING_OPERATION_H

#include <cstddef>
#include <memory>
#include <vector>

#include "geometryIf.h"
#include "toolPathIf.h"

namespace transmuter::pathgenerator::algorithms
{

/** Owns the state and cutting mechanics for one unidirectional three-axis raster run. */
class ThreeAxisRasterCuttingOperation
{
   public:
    struct Layer
    {
        double toolZ;
    };

    struct Row
    {
        double toolY;
    };

    ThreeAxisRasterCuttingOperation(const geometry::GeometryIf& stock,
        const geometry::GeometryIf& model, const geometry::GeometryIf& tool,
        double geometryTolerance, geometry::ToolPathIf& pathOutput);

    std::vector<Layer> layers(double layerSpacing) const;
    std::vector<Row> rows(double passSpacing) const;
    double rowStartX() const;
    double rowEndX() const;
    void prepareToCut(const Layer& layer);
    void beginRow(const Row& row);
    void moveToolAlongRowAvoidingModel(double toolX);
    void finishCurrentRow();
    void finishCurrentLayer();

    size_t completedLayerCount() const;
    std::shared_ptr<geometry::GeometryIf> takeWorkpiece();

   private:
    struct VerticalToolEnvelope
    {
        double radius;
        double minimumLocalZ;
    };

    void setupGeneration(const geometry::GeometryIf& stock,
        const geometry::GeometryIf& model, const geometry::GeometryIf& tool,
        double geometryTolerance);
    VerticalToolEnvelope calculateToolEnvelope(const geometry::GeometryIf& tool) const;
    Eigen::Vector3d liftToAvoidModel(const Eigen::Vector3d& requestedPosition) const;
    bool toolIntersectsModelAt(const Eigen::Vector3d& position) const;
    Eigen::Affine3d toolTransform(const geometry::Vertex& pose) const;
    void cutWorkpieceAtCurrentRowPoses();
    void retractToolToSafeHeight();
    void moveToolAboveCurrentRowStart();
    geometry::Vertex appendPathPoint(const Eigen::Vector3d& position,
        geometry::MotionSpeed incomingSpeed = geometry::MotionSpeed::CUTTING);

    std::shared_ptr<geometry::GeometryIf> m_workpiece;
    geometry::ToolPathIf& m_pathOutput;
    const geometry::GeometryIf* m_modelGeometry = nullptr;
    const geometry::GeometryIf* m_toolGeometry = nullptr;
    geometry::GeometryBounds m_stockBounds;
    VerticalToolEnvelope m_tool;
    std::vector<geometry::Vertex> m_currentRowCuttingPoses;
    Eigen::Vector3d m_currentPosition = Eigen::Vector3d::Zero();
    double m_rowStartX = 0.0;
    double m_rowEndX = 0.0;
    double m_safeToolOriginZ = 0.0;
    double m_layerZ = 0.0;
    double m_rowY = 0.0;
    double m_geometryTolerance = 0.0;
    size_t m_completedLayerCount = 0;
    size_t m_pointsSincePublicationPause = 0;
    bool m_hasCurrentPosition = false;
};

}  // namespace transmuter::pathgenerator::algorithms

#endif  // TRANSMUTER_PATHGEN_THREE_AXIS_RASTER_CUTTING_OPERATION_H
