#include "threeAxisRasterCuttingOperation.h"

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

#include "trace.h"

namespace transmuter::pathgenerator::algorithms
{

namespace
{

constexpr size_t PATH_PUBLICATION_BATCH_SIZE = 128;
constexpr auto PATH_PUBLICATION_PAUSE = std::chrono::milliseconds(2);

}  // namespace

ThreeAxisRasterCuttingOperation::ThreeAxisRasterCuttingOperation(
    const geometry::GeometryIf& stock, const geometry::GeometryIf& model,
    const geometry::GeometryIf& tool, double geometryTolerance,
    geometry::ToolPathIf& pathOutput)
    : m_pathOutput(pathOutput)
{
    setupGeneration(stock, model, tool, geometryTolerance);
}

void ThreeAxisRasterCuttingOperation::setupGeneration(const geometry::GeometryIf& stock,
    const geometry::GeometryIf& model, const geometry::GeometryIf& tool,
    double geometryTolerance)
{
    m_pathOutput.clear();
    m_workpiece = stock.clone();
    m_modelGeometry = &model;
    m_toolGeometry = &tool;
    m_geometryTolerance = geometryTolerance;
    m_stockBounds = stock.bounds();
    m_tool = calculateToolEnvelope(tool);
    m_rowStartX = m_stockBounds.min.x() - m_tool.radius;
    m_rowEndX = m_stockBounds.max.x() + m_tool.radius;
    m_safeToolOriginZ =
        m_stockBounds.max.z() + m_geometryTolerance - m_tool.minimumLocalZ;
    INFO("Prepared +Z raster cutting operation: X range "
         << m_rowStartX << " to " << m_rowEndX << " mm, safe tool-origin Z "
         << m_safeToolOriginZ << " mm.");
}

ThreeAxisRasterCuttingOperation::VerticalToolEnvelope
ThreeAxisRasterCuttingOperation::calculateToolEnvelope(
    const geometry::GeometryIf& tool) const
{
    return VerticalToolEnvelope{
        .radius = tool.maximumDistanceFromAxis(
            Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ()),
        .minimumLocalZ = tool.bounds().min.z(),
    };
}

std::vector<ThreeAxisRasterCuttingOperation::Layer>
ThreeAxisRasterCuttingOperation::layers(double layerSpacing) const
{
    const double bottomLayerZ = m_stockBounds.min.z() - m_tool.minimumLocalZ;
    double layerZ = m_stockBounds.max.z() - m_tool.minimumLocalZ;

    std::vector<Layer> layers;
    while (true)
    {
        layers.push_back(Layer{.toolZ = layerZ});
        if (layerZ <= bottomLayerZ + m_geometryTolerance)
        {
            return layers;
        }
        layerZ = std::max(bottomLayerZ, layerZ - layerSpacing);
    }
}

std::vector<ThreeAxisRasterCuttingOperation::Row>
ThreeAxisRasterCuttingOperation::rows(double passSpacing) const
{
    const double stepOver =
        std::min(passSpacing, std::max(m_tool.radius * 2.0, m_geometryTolerance));
    const double firstRowY = m_stockBounds.min.y() - m_tool.radius;
    const double lastRowY = m_stockBounds.max.y() + m_tool.radius;

    std::vector<Row> rows;
    for (double rowY = firstRowY; rowY < lastRowY - m_geometryTolerance;
         rowY += stepOver)
    {
        rows.push_back(Row{.toolY = rowY});
    }
    rows.push_back(Row{.toolY = lastRowY});
    return rows;
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

void ThreeAxisRasterCuttingOperation::beginRow(const Row& row)
{
    m_rowY = row.toolY;
    retractToolToSafeHeight();
    moveToolAboveCurrentRowStart();
}

void ThreeAxisRasterCuttingOperation::moveToolAboveCurrentRowStart()
{
    appendPathPoint(Eigen::Vector3d(m_rowStartX, m_rowY, m_safeToolOriginZ),
        geometry::MotionSpeed::RAPID);
}

void ThreeAxisRasterCuttingOperation::moveToolAlongRowAvoidingModel(double toolX)
{
    const Eigen::Vector3d requestedPosition(toolX, m_rowY, m_layerZ);
    const Eigen::Vector3d collisionFreePosition = liftToAvoidModel(requestedPosition);
    m_currentRowCuttingPoses.push_back(appendPathPoint(collisionFreePosition));
}

void ThreeAxisRasterCuttingOperation::finishCurrentRow()
{
    cutWorkpieceAtCurrentRowPoses();
    m_currentRowCuttingPoses.clear();
}

void ThreeAxisRasterCuttingOperation::finishCurrentLayer()
{
    retractToolToSafeHeight();
    ++m_completedLayerCount;
}

void ThreeAxisRasterCuttingOperation::cutWorkpieceAtCurrentRowPoses()
{
    std::vector<Eigen::Affine3d> toolTransforms;
    toolTransforms.reserve(m_currentRowCuttingPoses.size());
    for (const auto& pose : m_currentRowCuttingPoses)
    {
        toolTransforms.push_back(toolTransform(pose));
    }
    m_workpiece->cut(*m_toolGeometry, toolTransforms);
}

Eigen::Affine3d ThreeAxisRasterCuttingOperation::toolTransform(
    const geometry::Vertex& pose) const
{
    if (pose.normal.isZero())
    {
        ERROR("A tool pose must have a non-zero direction.");
    }

    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.linear() = Eigen::Quaterniond::FromTwoVectors(
                             Eigen::Vector3d::UnitZ(), pose.normal.normalized())
                             .toRotationMatrix();
    transform.translation() = pose.pos;
    return transform;
}

Eigen::Vector3d ThreeAxisRasterCuttingOperation::liftToAvoidModel(
    const Eigen::Vector3d& requestedPosition) const
{
    if (!toolIntersectsModelAt(requestedPosition))
    {
        return requestedPosition;
    }

    Eigen::Vector3d safePosition(
        requestedPosition.x(), requestedPosition.y(), m_safeToolOriginZ);
    Eigen::Vector3d collidingPosition = requestedPosition;
    while (safePosition.z() - collidingPosition.z() > m_geometryTolerance)
    {
        Eigen::Vector3d candidate = collidingPosition;
        candidate.z() = (collidingPosition.z() + safePosition.z()) * 0.5;
        if (toolIntersectsModelAt(candidate))
        {
            collidingPosition = candidate;
        }
        else
        {
            safePosition = candidate;
        }
    }
    return safePosition;
}

bool ThreeAxisRasterCuttingOperation::toolIntersectsModelAt(
    const Eigen::Vector3d& position) const
{
    const geometry::Vertex pose{
        .pos = position,
        .normal = Eigen::Vector3d::UnitZ(),
    };
    return m_modelGeometry->intersects(*m_toolGeometry, toolTransform(pose));
}

void ThreeAxisRasterCuttingOperation::retractToolToSafeHeight()
{
    if (!m_hasCurrentPosition ||
        m_currentPosition.z() >= m_safeToolOriginZ - m_geometryTolerance)
    {
        return;
    }

    Eigen::Vector3d safePosition = m_currentPosition;
    safePosition.z() = m_safeToolOriginZ;
    appendPathPoint(safePosition);
}

geometry::Vertex ThreeAxisRasterCuttingOperation::appendPathPoint(
    const Eigen::Vector3d& position, geometry::MotionSpeed incomingSpeed)
{
    const geometry::Vertex pathPoint{
        .pos = position,
        .normal = Eigen::Vector3d::UnitZ(),
    };
    m_pathOutput.append(pathPoint, incomingSpeed);
    m_currentPosition = position;
    m_hasCurrentPosition = true;

    ++m_pointsSincePublicationPause;
    if (m_pointsSincePublicationPause >= PATH_PUBLICATION_BATCH_SIZE)
    {
        // Keep the shared tool path responsive and visibly growing during generation.
        std::this_thread::sleep_for(PATH_PUBLICATION_PAUSE);
        m_pointsSincePublicationPause = 0;
    }
    return pathPoint;
}

size_t ThreeAxisRasterCuttingOperation::completedLayerCount() const
{
    return m_completedLayerCount;
}

std::shared_ptr<geometry::GeometryIf> ThreeAxisRasterCuttingOperation::takeWorkpiece()
{
    return std::move(m_workpiece);
}

}  // namespace transmuter::pathgenerator::algorithms
