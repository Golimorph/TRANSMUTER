#include "threeAxisSpiralCuttingOperation.h"

#include <algorithm>
#include <chrono>
#include <thread>

#include "trace.h"

namespace transmuter::pathgenerator::algorithms
{

namespace
{

constexpr size_t PATH_PUBLICATION_BATCH_SIZE = 128;
constexpr auto PATH_PUBLICATION_PAUSE = std::chrono::milliseconds(2);

}  // namespace

ThreeAxisSpiralCuttingOperation::ThreeAxisSpiralCuttingOperation(const geometry::GeometryIf& stock,
    const geometry::GeometryIf& model, const geometry::GeometryIf& tool, double geometryTolerance,
    geometry::ToolPathIf& pathOutput)
    : m_pathOutput(pathOutput)
{
    setupGeneration(stock, model, tool, geometryTolerance);
}

void ThreeAxisSpiralCuttingOperation::setupGeneration(const geometry::GeometryIf& stock,
    const geometry::GeometryIf& model, const geometry::GeometryIf& tool, double geometryTolerance)
{
    m_pathOutput.clear();
    m_workpiece = stock.clone();
    m_modelGeometry = &model;
    m_toolGeometry = &tool;
    m_geometryTolerance = geometryTolerance;
    m_stockBounds = stock.bounds();
    m_tool = calculateToolEnvelope(tool);
    m_center = (m_stockBounds.min.head<2>() + m_stockBounds.max.head<2>()) / 2.0;
    m_outerRadius = calculateOuterRadius(stock, m_tool.radius);
    m_safeToolOriginZ = m_stockBounds.max.z() + m_geometryTolerance - m_tool.minimumLocalZ;
    INFO("Prepared +Z cutting operation: XY center (" << m_center.x() << ", " << m_center.y()
                                                        << ") mm, outer tool-center radius " << m_outerRadius
                                                        << " mm, safe tool-origin Z " << m_safeToolOriginZ << " mm.");
}

ThreeAxisSpiralCuttingOperation::VerticalToolEnvelope ThreeAxisSpiralCuttingOperation::calculateToolEnvelope(const geometry::GeometryIf& tool) const
{
    return VerticalToolEnvelope{
        .radius = tool.maximumDistanceFromAxis(Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ()),
        .minimumLocalZ = tool.bounds().min.z(),
    };
}

double ThreeAxisSpiralCuttingOperation::calculateOuterRadius(const geometry::GeometryIf& stock, double toolRadius) const
{
    const Eigen::Vector3d axisPoint(m_center.x(), m_center.y(), 0.0);
    return stock.maximumDistanceFromAxis(axisPoint, Eigen::Vector3d::UnitZ()) + toolRadius;
}

std::vector<ThreeAxisSpiralCuttingOperation::Layer> ThreeAxisSpiralCuttingOperation::layers(double layerSpacing) const
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

std::vector<ThreeAxisSpiralCuttingOperation::Circle> ThreeAxisSpiralCuttingOperation::concentricCircles(double passSpacing) const
{
    const double radialStepOver = std::min(passSpacing, std::max(m_tool.radius * 2.0, m_geometryTolerance));
    std::vector<Circle> circles;
    for (double radius = m_outerRadius; radius > m_geometryTolerance; radius -= radialStepOver)
    {
        circles.push_back(Circle{.radius = radius});
    }
    return circles;
}

void ThreeAxisSpiralCuttingOperation::prepareToCut(const Layer& layer)
{
    if (m_completedLayerCount > 0)
    {
        retractToolToSafeHeight();
        m_layerZ = layer.toolZ;
        moveToolToNextLayerStart();
        return;
    }
    m_layerZ = layer.toolZ;
}

Eigen::Vector2d ThreeAxisSpiralCuttingOperation::center() const
{
    return m_center;
}

void ThreeAxisSpiralCuttingOperation::finishCurrentLayerAtCenter()
{
    moveToolToLayerHeightAvoidingModel(m_center);
    cutWorkpieceAtCurrentPoses();
    m_currentLayerCuttingPoses.clear();
    ++m_completedLayerCount;
}

void ThreeAxisSpiralCuttingOperation::cutWorkpieceAtCurrentPoses()
{
    std::vector<Eigen::Affine3d> toolTransforms;
    toolTransforms.reserve(m_currentLayerCuttingPoses.size());
    for (const auto& pose : m_currentLayerCuttingPoses)
    {
        toolTransforms.push_back(toolTransform(pose));
    }
    m_workpiece->cut(*m_toolGeometry, toolTransforms);
}

Eigen::Affine3d ThreeAxisSpiralCuttingOperation::toolTransform(const geometry::Vertex& pose) const
{
    if (pose.normal.isZero())
    {
        ERROR("A tool pose must have a non-zero direction.");
    }

    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), pose.normal.normalized())
                             .toRotationMatrix();
    transform.translation() = pose.pos;
    return transform;
}

Eigen::Vector3d ThreeAxisSpiralCuttingOperation::liftToAvoidModel(const Eigen::Vector3d& requestedPosition) const
{
    if (!toolIntersectsModelAt(requestedPosition))
    {
        return requestedPosition;
    }

    Eigen::Vector3d safePosition(requestedPosition.x(), requestedPosition.y(), m_safeToolOriginZ);
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

bool ThreeAxisSpiralCuttingOperation::toolIntersectsModelAt(const Eigen::Vector3d& position) const
{
    const geometry::Vertex pose{.pos = position, .normal = Eigen::Vector3d::UnitZ()};
    return m_modelGeometry->intersects(*m_toolGeometry, toolTransform(pose));
}

void ThreeAxisSpiralCuttingOperation::moveToolToLayerHeightAvoidingModel(const Eigen::Vector2d& toolCenter)
{
    const Eigen::Vector3d requestedPosition(toolCenter.x(), toolCenter.y(), m_layerZ);
    const Eigen::Vector3d collisionFreePosition = liftToAvoidModel(requestedPosition);
    m_currentLayerCuttingPoses.push_back(appendPathPoint(collisionFreePosition));
}

void ThreeAxisSpiralCuttingOperation::retractToolToSafeHeight()
{
    appendPathPoint(Eigen::Vector3d(m_center.x(), m_center.y(), m_safeToolOriginZ));
}

void ThreeAxisSpiralCuttingOperation::moveToolToNextLayerStart()
{
    appendPathPoint(
        Eigen::Vector3d(m_center.x() + m_outerRadius, m_center.y(), m_safeToolOriginZ),
        geometry::MotionSpeed::RAPID);
    appendPathPoint(Eigen::Vector3d(m_center.x() + m_outerRadius, m_center.y(), m_layerZ));
}

geometry::Vertex ThreeAxisSpiralCuttingOperation::appendPathPoint(
    const Eigen::Vector3d& position, geometry::MotionSpeed incomingSpeed)
{
    geometry::Vertex pathPoint;
    pathPoint.pos = position;
    pathPoint.normal = Eigen::Vector3d::UnitZ();
    m_pathOutput.append(pathPoint, incomingSpeed);

    ++m_pointsSincePublicationPause;
    if (m_pointsSincePublicationPause >= PATH_PUBLICATION_BATCH_SIZE)
    {
        // Keep the shared tool path responsive and visibly growing during generation.
        std::this_thread::sleep_for(PATH_PUBLICATION_PAUSE);
        m_pointsSincePublicationPause = 0;
    }
    return pathPoint;
}

size_t ThreeAxisSpiralCuttingOperation::completedLayerCount() const
{
    return m_completedLayerCount;
}

std::shared_ptr<geometry::GeometryIf> ThreeAxisSpiralCuttingOperation::takeWorkpiece()
{
    return std::move(m_workpiece);
}

}  // namespace transmuter::pathgenerator::algorithms
