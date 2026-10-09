#include "ThreeAxisSpiralCuttingOperation.h"

#include <algorithm>

#include "toolPose.h"
#include "trace.h"

namespace transmuter::pathgenerator::algorithms
{

ThreeAxisSpiralCuttingOperation::ThreeAxisSpiralCuttingOperation(const geometry::GeometryIf& stock, const geometry::GeometryIf& model, const geometry::GeometryIf& tool, double geometryTolerance, geometry::GeometryIf& workpiece, MachineKinematicsIf& machine, const std::atomic<bool>& stop)
    : m_modelGeometry(model), m_toolGeometry(tool), m_workpiece(workpiece), m_machine(machine), m_stop(stop)
{
    setupGeneration(stock, geometryTolerance);
}

void ThreeAxisSpiralCuttingOperation::setupGeneration(const geometry::GeometryIf& stock, double geometryTolerance)
{
    m_geometryTolerance = geometryTolerance;
    m_stockBounds = stock.bounds();
    m_tool = calculateToolEnvelope(m_toolGeometry);
    m_center = (m_stockBounds.min.head<2>() + m_stockBounds.max.head<2>()) / 2.0;
    m_outerRadius = calculateOuterRadius(stock, m_tool.radius);
    m_safeToolOriginZ = std::max(m_stockBounds.max.z(), m_modelGeometry.bounds().max.z()) + m_geometryTolerance -
                        m_tool.minimumLocalZ;
    INFO("Prepared +Z cutting operation: XY center (" << m_center.x() << ", " << m_center.y() << ") mm, outer tool-center radius " << m_outerRadius << " mm, safe tool-origin Z " << m_safeToolOriginZ << " mm.");
}

ThreeAxisSpiralCuttingOperation::VerticalToolEnvelope ThreeAxisSpiralCuttingOperation::calculateToolEnvelope(const geometry::GeometryIf& tool)
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
    ++m_completedLayerCount;
}

geometry::Vertex ThreeAxisSpiralCuttingOperation::liftToAvoidModel(const geometry::Vertex& requestedPose) const
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

bool ThreeAxisSpiralCuttingOperation::toolIntersectsModelAt(const geometry::Vertex& pose) const
{
    auto transform = geometry::toolTransform(pose);
    return m_modelGeometry.intersects(m_toolGeometry, transform);
}

void ThreeAxisSpiralCuttingOperation::moveToolToLayerHeightAvoidingModel(const Eigen::Vector2d& toolCenter)
{
    const auto requestedPose = verticalToolPose(toolCenter.x(), toolCenter.y(), m_layerZ);
    const auto collisionFreePose = liftToAvoidModel(requestedPose);
    moveToolTo(collisionFreePose, MotionType::CUTTING);
}

void ThreeAxisSpiralCuttingOperation::moveToolTo(const geometry::Vertex& pose, MotionType motionType)
{
    const auto result = m_machine.propose(pose, motionType, m_stop);
    if (result.status == MachinePathStatus::CANCELLED)
    {
        return;
    }
    if (result.status != MachinePathStatus::ACCEPTED)
    {
        ERROR("Machine rejected spiral move to " << pose.pos.transpose() << ".");
    }
    if (motionType == MotionType::CUTTING)
    {
        m_workpiece.cut(m_toolGeometry, geometry::toolTransform(pose));
    }
}

geometry::Vertex ThreeAxisSpiralCuttingOperation::verticalToolPose(double x, double y, double z)
{
    return {Eigen::Vector3d(x, y, z), Eigen::Vector3d::UnitZ()};
}

void ThreeAxisSpiralCuttingOperation::retractToolToSafeHeight()
{
    const auto pose = verticalToolPose(m_center.x(), m_center.y(), m_safeToolOriginZ);
    moveToolTo(pose, MotionType::RAPID);
}

void ThreeAxisSpiralCuttingOperation::moveToolToNextLayerStart()
{
    const auto layerStart = verticalToolPose(m_center.x() + m_outerRadius, m_center.y(), m_safeToolOriginZ);
    moveToolTo(layerStart, MotionType::RAPID);

    const auto plunge = verticalToolPose(m_center.x() + m_outerRadius, m_center.y(), m_layerZ);
    moveToolTo(plunge, MotionType::CUTTING);
}

size_t ThreeAxisSpiralCuttingOperation::completedLayerCount() const
{
    return m_completedLayerCount;
}

}  // namespace transmuter::pathgenerator::algorithms
