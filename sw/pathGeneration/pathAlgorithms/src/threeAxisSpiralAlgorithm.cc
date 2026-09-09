#include "threeAxisSpiralAlgorithm.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <utility>
#include <vector>

#include "trace.h"

namespace transmuter::pathgenerator::algorithms
{

namespace
{

constexpr float TWO_PI = 2.0f * static_cast<float>(M_PI);
constexpr float MINIMUM_INCREMENT = 0.001f;
constexpr float VERTICAL_CLEARANCE = 0.01f;
constexpr size_t PATH_PUBLICATION_BATCH_SIZE = 128;
constexpr auto PATH_PUBLICATION_PAUSE = std::chrono::milliseconds(2);

}  // namespace

ThreeAxisSpiralAlgorithm::ThreeAxisSpiralAlgorithm(const transmuter::geometry::MillingScene& scene)
    : PathAlgorithmIf(scene)
{
}

void ThreeAxisSpiralAlgorithm::setConfig(const PathGeneratorConfig& config)
{
    m_config = config;
    INFO("SpiralAlgo: Configuration updated.");
}

PathGenerationResult ThreeAxisSpiralAlgorithm::generate(
    std::atomic<bool>& stopRequested,
    std::shared_ptr<geometry::PointCloud> liveToolPath)
{
    if (!liveToolPath)
    {
        ERROR("toolPath is null ptr");
    }

    GenerationContext context = setupGeneration(std::move(liveToolPath));
    while (!stopRequested.load())
    {
        if (context.layerCount > 0)
        {
            appendLayerTransition(context);
        }

        cutLayer(context);

        if (!advanceToNextLayer(context))
        {
            break;
        }
    }
    INFO("SpiralAlgo: Generated " << context.layerCount
                                  << " counter-clockwise layers and simulated "
                                  << context.removedPointCount
                                  << " points from the local workpiece.");

    return PathGenerationResult{.toolPath = std::move(context.toolPath),
                                .workpiece = std::move(context.workpiece)};
}

ThreeAxisSpiralAlgorithm::GenerationContext ThreeAxisSpiralAlgorithm::setupGeneration(
    std::shared_ptr<geometry::PointCloud> liveToolPath)
{
    GenerationContext context;
    context.toolPath = std::move(liveToolPath);
    context.toolPath->clear();
    m_pointsSincePublicationPause = 0;

    // The local workpiece remains private until generate returns its completed result.
    context.workpiece = stock().clone();
    context.materialRemoval = std::make_unique<geometry::MaterialRemoval>(toolModel());
    context.stockBounds = PointCloudCollisionDetector::calculateBounds(stock());
    context.modelBounds = PointCloudCollisionDetector::calculateBounds(model());
    context.tool = calculateToolEnvelope();
    context.center =
        (context.stockBounds.min.head<2>() + context.stockBounds.max.head<2>()) * 0.5f;
    context.outerRadius = calculateOuterRadius(context.center, context.tool.radius);
    context.radialStepOver =
        std::min(static_cast<float>(m_config.passSpacing), context.tool.radius * 2.0f);
    context.pathSampleDistance = static_cast<float>(m_config.pointSpacing);
    context.layerStepDown = static_cast<float>(m_config.layerSpacing);
    context.collisionDetector = std::make_unique<PointCloudCollisionDetector>(model());
    context.safeTravelZ = std::max(context.stockBounds.max.z(), context.modelBounds.max.z()) +
                          context.collisionDetector->interpolationRadius() + VERTICAL_CLEARANCE -
                          context.tool.minimumLocalZ;
    context.layerZ = context.stockBounds.max.z() - context.tool.minimumLocalZ;
    return context;
}

ThreeAxisSpiralAlgorithm::VerticalToolEnvelope ThreeAxisSpiralAlgorithm::calculateToolEnvelope()
    const
{
    VerticalToolEnvelope result{0.0f, std::numeric_limits<float>::max(),
                                std::numeric_limits<float>::lowest()};
    std::vector<geometry::Vertex> toolVertices;
    toolModel().copyTo(toolVertices);

    for (const auto& vertex : toolVertices)
    {
        result.radius = std::max(result.radius, vertex.pos.head<2>().norm());
        result.minimumLocalZ = std::min(result.minimumLocalZ, vertex.pos.z());
        result.maximumLocalZ = std::max(result.maximumLocalZ, vertex.pos.z());
    }

    return result;
}

float ThreeAxisSpiralAlgorithm::calculateOuterRadius(const Eigen::Vector2f& center,
                                                     float toolRadius) const
{
    std::vector<geometry::Vertex> stockVertices;
    stock().copyTo(stockVertices);

    float outerRadius = 0.0f;
    for (const auto& stockVertex : stockVertices)
    {
        outerRadius =
            std::max(outerRadius, (stockVertex.pos.head<2>() - center).norm());
    }
    return outerRadius + toolRadius;
}

void ThreeAxisSpiralAlgorithm::appendLayerTransition(const GenerationContext& context)
{
    appendPathPoint(*context.toolPath,
                    Eigen::Vector3f(context.center.x(), context.center.y(), context.safeTravelZ));
    appendPathPoint(*context.toolPath,
                    Eigen::Vector3f(context.center.x() + context.outerRadius, context.center.y(),
                                    context.safeTravelZ));
    appendPathPoint(*context.toolPath,
                    Eigen::Vector3f(context.center.x() + context.outerRadius, context.center.y(),
                                    context.layerZ));
}

void ThreeAxisSpiralAlgorithm::cutLayer(GenerationContext& context)
{
    const size_t sizeBeforeLayer = context.workpiece->size();
    std::vector<geometry::Vertex> cuttingPoses;
    appendSpiralPass(context, cuttingPoses);

    const float centerZ = context.collisionDetector->collisionFreeVerticalToolTipZ(
        context.center, context.layerZ, context.tool.radius, context.tool.minimumLocalZ,
        VERTICAL_CLEARANCE);
    cuttingPoses.push_back(appendPathPoint(
        *context.toolPath, Eigen::Vector3f(context.center.x(), context.center.y(), centerZ)));
    context.materialRemoval->apply(*context.workpiece, cuttingPoses);
    context.removedPointCount += sizeBeforeLayer - context.workpiece->size();
    ++context.layerCount;
}

void ThreeAxisSpiralAlgorithm::appendSpiralPass(
    const GenerationContext& context, std::vector<geometry::Vertex>& cuttingPoses)
{
    float angle = 0.0f;
    float radius = context.outerRadius;

    while (radius > MINIMUM_INCREMENT)
    {
        const Eigen::Vector2f toolCenter =
            context.center + Eigen::Vector2f(std::cos(angle), std::sin(angle)) * radius;
        const float collisionFreeZ = context.collisionDetector->collisionFreeVerticalToolTipZ(
            toolCenter, context.layerZ, context.tool.radius, context.tool.minimumLocalZ,
            VERTICAL_CLEARANCE);
        const Eigen::Vector3f position(toolCenter.x(), toolCenter.y(), collisionFreeZ);
        cuttingPoses.push_back(appendPathPoint(*context.toolPath, position));

        // Increasing angle is counter-clockwise when viewed along the vertical +Z axis.
        angle += context.pathSampleDistance /
                 std::max(radius, context.pathSampleDistance);
        radius = context.outerRadius - context.radialStepOver * angle / TWO_PI;
    }
}

bool ThreeAxisSpiralAlgorithm::advanceToNextLayer(GenerationContext& context) const
{
    const float bottomLayerZ = context.stockBounds.min.z() - context.tool.minimumLocalZ;
    if (context.layerZ <= bottomLayerZ + MINIMUM_INCREMENT)
    {
        return false;
    }

    context.layerZ = std::max(bottomLayerZ, context.layerZ - context.layerStepDown);
    return true;
}

geometry::Vertex ThreeAxisSpiralAlgorithm::appendPathPoint(geometry::PointCloud& toolPath,
                                                            const Eigen::Vector3f& position)
{
    geometry::Vertex pathPoint;
    pathPoint.pos = position;
    pathPoint.normal = Eigen::Vector3f::UnitZ();
    toolPath.addVertex(pathPoint);

    ++m_pointsSincePublicationPause;
    if (m_pointsSincePublicationPause >= PATH_PUBLICATION_BATCH_SIZE)
    {
        // Keep the shared tool path responsive and visibly growing during generation.
        std::this_thread::sleep_for(PATH_PUBLICATION_PAUSE);
        m_pointsSincePublicationPause = 0;
    }

    return pathPoint;
}

}  // namespace transmuter::pathgenerator::algorithms
