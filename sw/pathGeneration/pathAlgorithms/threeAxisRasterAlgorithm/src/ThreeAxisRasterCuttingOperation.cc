#include "ThreeAxisRasterCuttingOperation.h"

#include <algorithm>

#include "toolPose.h"
#include "trace.h"

namespace transmuter::pathgenerator::algorithms
{

ThreeAxisRasterCuttingOperation::ThreeAxisRasterCuttingOperation(const geometry::GeometryIf& stock, const geometry::GeometryIf& model, const geometry::GeometryIf& tool, double geometryTolerance, geometry::GeometryIf& workpiece, MachineKinematicsIf& machine, const std::atomic<bool>& stop)
    : m_modelGeometry(model), m_toolGeometry(tool), m_workpiece(workpiece), m_machine(machine), m_stop(stop)
{
    setupGeneration(stock, geometryTolerance);
}

void ThreeAxisRasterCuttingOperation::setupGeneration(const geometry::GeometryIf& stock, double geometryTolerance)
{
    m_geometryTolerance = geometryTolerance;
    m_stockBounds = stock.bounds();
    m_tool = calculateToolEnvelope(m_toolGeometry);
    m_rowStartX = m_stockBounds.min.x() - m_tool.radius;
    m_rowEndX = m_stockBounds.max.x() + m_tool.radius;
    m_safeToolOriginZ = std::max(m_stockBounds.max.z(), m_modelGeometry.bounds().max.z()) + m_geometryTolerance -
                        m_tool.minimumLocalZ;
    INFO("Prepared +Z raster cutting operation: X range " << m_rowStartX << " to " << m_rowEndX << " mm, safe tool-origin Z " << m_safeToolOriginZ << " mm.");
}

ThreeAxisRasterCuttingOperation::VerticalToolEnvelope ThreeAxisRasterCuttingOperation::calculateToolEnvelope(const geometry::GeometryIf& tool)
{
    return {
        .radius = tool.maximumDistanceFromAxis(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ()),
        .minimumLocalZ = tool.bounds().min.z(),
    };
}

std::vector<ThreeAxisRasterCuttingOperation::Layer> ThreeAxisRasterCuttingOperation::layers(double layerSpacing) const
{
    const double bottomLayerZ = m_stockBounds.min.z() - m_tool.minimumLocalZ;
    double layerZ = m_stockBounds.max.z() - m_tool.minimumLocalZ;
    std::vector<Layer> result;
    while (true)
    {
        result.push_back({layerZ});
        if (layerZ <= bottomLayerZ + m_geometryTolerance)
        {
            return result;
        }
        layerZ = std::max(bottomLayerZ, layerZ - layerSpacing);
    }
}

std::vector<ThreeAxisRasterCuttingOperation::Row> ThreeAxisRasterCuttingOperation::rows(double passSpacing) const
{
    const double stepOver = std::min(passSpacing, std::max(m_tool.radius * 2.0, m_geometryTolerance));
    const double firstRowY = m_stockBounds.min.y() - m_tool.radius;
    const double lastRowY = m_stockBounds.max.y() + m_tool.radius;
    std::vector<Row> result;
    for (double y = firstRowY; y < lastRowY - m_geometryTolerance; y += stepOver)
        result.push_back({y});
    result.push_back({lastRowY});
    return result;
}

double ThreeAxisRasterCuttingOperation::rowStartX() const
{
    return m_rowStartX;
}
double ThreeAxisRasterCuttingOperation::rowEndX() const
{
    return m_rowEndX;
}

void ThreeAxisRasterCuttingOperation::prepareToCut(const Layer& layer)
{
    m_layerZ = layer.toolZ;
}

void ThreeAxisRasterCuttingOperation::beginRow(const Row& row, bool firstRowInLayer)
{
    m_rowY = row.toolY;
    retractToolToSafeHeight();
    if (firstRowInLayer)
    {
        const auto preloadPosition = verticalToolPose(m_rowStartX - m_tool.radius, m_rowY - m_tool.radius, m_safeToolOriginZ);
        moveToolTo(preloadPosition, MotionType::RAPID);
    }
    const auto rowStart = verticalToolPose(m_rowStartX, m_rowY, m_safeToolOriginZ);
    moveToolTo(rowStart, MotionType::RAPID);
}

void ThreeAxisRasterCuttingOperation::moveToolAlongRowAvoidingModel(double toolX)
{
    const auto requestedPose = verticalToolPose(toolX, m_rowY, m_layerZ);
    const auto modelSafePose = liftToAvoidModel(requestedPose);
    moveToolTo(modelSafePose, MotionType::CUTTING);
}

void ThreeAxisRasterCuttingOperation::finishCurrentLayer()
{
    retractToolToSafeHeight();
    ++m_completedLayerCount;
}

geometry::Vertex ThreeAxisRasterCuttingOperation::liftToAvoidModel(const geometry::Vertex& requestedPose) const
{
    auto safePose = verticalToolPose(requestedPose.pos.x(), requestedPose.pos.y(), m_safeToolOriginZ);
    auto collidingPose = requestedPose;
    while (safePose.pos.z() - collidingPose.pos.z() > m_geometryTolerance)
    {
        auto candidate = collidingPose;
        candidate.pos.z() = (collidingPose.pos.z() + safePose.pos.z()) * 0.5;
        if (toolIntersectsModelAt(candidate))
        {
            collidingPose = candidate;
        }
        else
        {
            safePose = candidate;
        }
    }
    return safePose;
}

bool ThreeAxisRasterCuttingOperation::toolIntersectsModelAt(const geometry::Vertex& pose) const
{
    return m_modelGeometry.intersects(m_toolGeometry, geometry::toolTransform(pose));
}

geometry::Vertex ThreeAxisRasterCuttingOperation::verticalToolPose(double x, double y, double z)
{
    return {Eigen::Vector3d(x, y, z), Eigen::Vector3d::UnitZ()};
}

void ThreeAxisRasterCuttingOperation::moveToolTo(const geometry::Vertex& pose, MotionType motionType)
{
    const auto result = m_machine.propose(pose, motionType, m_stop);
    if (result.status == MachinePathStatus::CANCELLED)
    {
        return;
    }
    if (result.status != MachinePathStatus::ACCEPTED)
    {
        ERROR("Machine rejected raster move to " << pose.pos.transpose() << ".");
    }
    if (motionType == MotionType::CUTTING)
    {
        m_workpiece.cut(m_toolGeometry, geometry::toolTransform(pose));
    }
}

void ThreeAxisRasterCuttingOperation::retractToolToSafeHeight()
{
    const auto path = m_machine.path();
    const auto lastMove = path->last();
    if (!lastMove)
    {
        return;
    }
    auto safePosition = m_machine.displayPose(lastMove->axes).toolInWorkpiece.pos;
    safePosition.z() = m_safeToolOriginZ;
    const auto pose = verticalToolPose(safePosition.x(), safePosition.y(), safePosition.z());
    moveToolTo(pose, MotionType::RAPID);
}

size_t ThreeAxisRasterCuttingOperation::completedLayerCount() const
{
    return m_completedLayerCount;
}

}  // namespace transmuter::pathgenerator::algorithms
