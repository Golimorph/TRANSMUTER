#ifndef TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_ALGORITHM_H
#define TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_ALGORITHM_H

#include <cstddef>
#include <memory>
#include <vector>

#include "materialRemoval.h"
#include "pathAlgorithmIf.h"
#include "pointCloudCollisionDetector.h"

namespace transmuter::pathgenerator::algorithms
{

class ThreeAxisSpiralAlgorithm : public PathAlgorithmIf
{
   public:
    explicit ThreeAxisSpiralAlgorithm(const transmuter::geometry::MillingScene& scene);
    ~ThreeAxisSpiralAlgorithm() override = default;

    void setConfig(const PathGeneratorConfig& config) override;
    PathGenerationResult generate(
        std::atomic<bool>& stopRequested,
        std::shared_ptr<geometry::PointCloud> liveToolPath) override;

   private:
    struct VerticalToolEnvelope
    {
        float radius;
        float minimumLocalZ;
        float maximumLocalZ;
    };

    struct GenerationContext
    {
        std::shared_ptr<geometry::PointCloud> workpiece;
        std::shared_ptr<geometry::PointCloud> toolPath;
        std::unique_ptr<geometry::MaterialRemoval> materialRemoval;
        std::unique_ptr<PointCloudCollisionDetector> collisionDetector;
        PointCloudBounds stockBounds;
        PointCloudBounds modelBounds;
        VerticalToolEnvelope tool;
        Eigen::Vector2f center = Eigen::Vector2f::Zero();
        float outerRadius = 0.0f;
        float radialStepOver = 0.0f;
        float pathSampleDistance = 0.0f;
        float layerStepDown = 0.0f;
        float safeTravelZ = 0.0f;
        float layerZ = 0.0f;
        size_t layerCount = 0;
        size_t removedPointCount = 0;
    };

    GenerationContext setupGeneration(
        std::shared_ptr<geometry::PointCloud> liveToolPath);
    VerticalToolEnvelope calculateToolEnvelope() const;
    float calculateOuterRadius(const Eigen::Vector2f& center, float toolRadius) const;
    void appendLayerTransition(const GenerationContext& context);
    void cutLayer(GenerationContext& context);
    void appendSpiralPass(const GenerationContext& context,
                          std::vector<geometry::Vertex>& cuttingPoses);
    bool advanceToNextLayer(GenerationContext& context) const;
    geometry::Vertex appendPathPoint(geometry::PointCloud& toolPath,
                                     const Eigen::Vector3f& position);

    PathGeneratorConfig m_config;
    size_t m_pointsSincePublicationPause = 0;
};

}  // namespace transmuter::pathgenerator::algorithms

#endif  // TRANSMUTER_PATHGEN_THREE_AXIS_SPIRAL_ALGORITHM_H
