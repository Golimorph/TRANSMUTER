#include "threeAxisSpiralCuttingOperation.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <thread>
#include <utility>

#include "trace.h"

namespace transmuter::pathgenerator::algorithms
{

namespace
{

constexpr float TWO_PI = 2.0f * std::numbers::pi_v<float>;
constexpr float MINIMUM_INCREMENT = 0.001f;
constexpr float VERTICAL_CLEARANCE = 0.01f;
constexpr size_t PATH_PUBLICATION_BATCH_SIZE = 128;
constexpr auto PATH_PUBLICATION_PAUSE = std::chrono::milliseconds(2);

}  // namespace

ThreeAxisSpiralCuttingOperation::ThreeAxisSpiralCuttingOperation(
    const geometry::GeometryIf& stock, const geometry::GeometryIf& model,
    const geometry::GeometryIf& tool,
    std::shared_ptr<geometry::PointCloud> liveToolPath)
{
    setupGeneration(stock, model, tool, std::move(liveToolPath));
}

void ThreeAxisSpiralCuttingOperation::setupGeneration(
    const geometry::GeometryIf& stock, const geometry::GeometryIf& model,
    const geometry::GeometryIf& tool,
    std::shared_ptr<geometry::PointCloud> liveToolPath)
{
    if (!liveToolPath)
    {
        ERROR("toolPath is null ptr");
    }

    m_toolPath = std::move(liveToolPath);
    m_toolPath->clear();
    m_workpiece = stock.clone();
    m_materialRemoval = geometry::createMaterialRemoval(*m_workpiece, tool);
    m_modelClearance = geometry::createVerticalToolClearance(model, tool);
    m_stockBounds = stock.bounds();
    m_tool = calculateToolEnvelope(tool);
    m_center = (m_stockBounds.min.head<2>() + m_stockBounds.max.head<2>()) * 0.5f;
    m_outerRadius = calculateOuterRadius(stock, m_tool.radius);
    m_safeTravelZ = std::max(m_stockBounds.max.z(), model.bounds().max.z()) +
                    m_modelClearance->geometryTolerance() + VERTICAL_CLEARANCE -
                    m_tool.minimumLocalZ;
}

ThreeAxisSpiralCuttingOperation::VerticalToolEnvelope
ThreeAxisSpiralCuttingOperation::calculateToolEnvelope(
    const geometry::GeometryIf& tool) const
{
    return VerticalToolEnvelope{
        .radius = tool.maximumDistanceFromAxis(Eigen::Vector3f::Zero(),
                                               Eigen::Vector3f::UnitZ()),
        .minimumLocalZ = tool.bounds().min.z(),
    };
}

float ThreeAxisSpiralCuttingOperation::calculateOuterRadius(
    const geometry::GeometryIf& stock, float toolRadius) const
{
    const Eigen::Vector3f axisPoint(m_center.x(), m_center.y(), 0.0f);
    return stock.maximumDistanceFromAxis(axisPoint, Eigen::Vector3f::UnitZ()) + toolRadius;
}

std::vector<ThreeAxisSpiralCuttingOperation::Layer>
ThreeAxisSpiralCuttingOperation::layersFromTopToBottom(
    double layerSpacing) const
{
    const float layerStepDown = static_cast<float>(layerSpacing);
    const float bottomLayerZ = m_stockBounds.min.z() - m_tool.minimumLocalZ;
    float layerZ = m_stockBounds.max.z() - m_tool.minimumLocalZ;

    std::vector<Layer> layers;
    while (true)
    {
        layers.push_back(Layer{.toolZ = layerZ});
        if (layerZ <= bottomLayerZ + MINIMUM_INCREMENT)
        {
            return layers;
        }
        layerZ = std::max(bottomLayerZ, layerZ - layerStepDown);
    }
}

std::vector<ThreeAxisSpiralCuttingOperation::Circle>
ThreeAxisSpiralCuttingOperation::concentricCircles(
    double passSpacing) const
{
    const float radialStepOver =
        std::min(static_cast<float>(passSpacing),
                 std::max(m_tool.radius * 2.0f, MINIMUM_INCREMENT));
    std::vector<Circle> circles;
    for (float radius = m_outerRadius; radius > MINIMUM_INCREMENT;
         radius -= radialStepOver)
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

void ThreeAxisSpiralCuttingOperation::cutCounterClockwise(
    const Circle& circle, double pointSpacing)
{
    const float circumference = TWO_PI * circle.radius;
    const float pathSampleDistance = static_cast<float>(pointSpacing);
    const size_t sampleCount = std::max<size_t>(
        3, static_cast<size_t>(std::ceil(circumference / pathSampleDistance)));

    for (size_t sample = 0; sample <= sampleCount; ++sample)
    {
        // Increasing angle is counter-clockwise when viewed along the vertical +Z axis.
        const float angle = TWO_PI * static_cast<float>(sample) /
                            static_cast<float>(sampleCount);
        const Eigen::Vector2f toolCenter =
            m_center + Eigen::Vector2f(std::cos(angle), std::sin(angle)) * circle.radius;
        moveToolToLayerHeightAvoidingModel(toolCenter);
    }
}

void ThreeAxisSpiralCuttingOperation::finishCurrentLayerAtCenter()
{
    moveToolToLayerHeightAvoidingModel(m_center);
    m_materialRemoval->apply(*m_workpiece, m_currentLayerCuttingPoses);
    m_currentLayerCuttingPoses.clear();
    ++m_completedLayerCount;
}

void ThreeAxisSpiralCuttingOperation::moveToolToLayerHeightAvoidingModel(
    const Eigen::Vector2f& toolCenter)
{
    // Starting each move at the layer height lowers the tool again as soon as clearance permits.
    const Eigen::Vector3f requestedPosition(toolCenter.x(), toolCenter.y(), m_layerZ);
    const Eigen::Vector3f collisionFreePosition =
        m_modelClearance->liftToAvoidCollision(requestedPosition, VERTICAL_CLEARANCE);
    m_currentLayerCuttingPoses.push_back(appendPathPoint(collisionFreePosition));
}

void ThreeAxisSpiralCuttingOperation::retractToolToSafeHeight()
{
    appendPathPoint(Eigen::Vector3f(m_center.x(), m_center.y(), m_safeTravelZ));
}

void ThreeAxisSpiralCuttingOperation::moveToolToNextLayerStart()
{
    appendPathPoint(
        Eigen::Vector3f(m_center.x() + m_outerRadius, m_center.y(), m_safeTravelZ));
    appendPathPoint(Eigen::Vector3f(m_center.x() + m_outerRadius, m_center.y(), m_layerZ));
}

geometry::Vertex ThreeAxisSpiralCuttingOperation::appendPathPoint(
    const Eigen::Vector3f& position)
{
    geometry::Vertex pathPoint;
    pathPoint.pos = position;
    pathPoint.normal = Eigen::Vector3f::UnitZ();
    m_toolPath->addVertex(pathPoint);

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

PathGenerationResult ThreeAxisSpiralCuttingOperation::takeResult()
{
    return PathGenerationResult{.toolPath = std::move(m_toolPath),
                                .workpiece = std::move(m_workpiece)};
}

}  // namespace transmuter::pathgenerator::algorithms
