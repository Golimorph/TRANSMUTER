#include "threeAxisSpiralAlgorithm.h"

#include <chrono>
#include <thread>

#include "trace.h"

namespace transmuter::pathgenerator::algorithms
{

ThreeAxisSpiralAlgorithm::ThreeAxisSpiralAlgorithm(const transmuter::geometry::MillingScene& scene)
    : PathAlgorithmIf(scene)
{
}

void ThreeAxisSpiralAlgorithm::setConfig(const PathGeneratorConfig& config)
{
    m_config = config;
    INFO("SpiralAlgo: Configuration updated.");
}

void ThreeAxisSpiralAlgorithm::generate(std::atomic<bool>& stopSignal)
{
    if (p_workPiece->isEmpty()) return;

    p_toolPath->clear();

    // 1. Find the bounds of the box to know where the "edges" are
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();

    for (size_t i = 0; i < p_workPiece->size(); ++i)
    {
        const auto& v = p_workPiece->getVertex(i);
        minX = std::min(minX, v.pos.x());
        maxX = std::max(maxX, v.pos.x());
        minY = std::min(minY, v.pos.y());
        maxY = std::max(maxY, v.pos.y());
    }

    // 2. Define the "Corners" of our path (at a specific Z height)
    float zHeight = 0.0f;
    std::vector<Eigen::Vector3f> corners = {
        {minX, minY, zHeight},
        {maxX, minY, zHeight},
        {maxX, maxY, zHeight},
        {minX, maxY, zHeight},
        {minX, minY, zHeight}  // Close the loop
    };

    // 3. Define the normals for each face we are traversing
    std::vector<Eigen::Vector3f> faceNormals = {{0, -1, 0}, {1, 0, 0}, {0, 1, 0}, {-1, 0, 0}};

    // 4. Interpolate between corners to create a smooth milling path
    const int segmentsPerEdge = 100;
    for (size_t i = 0; i < faceNormals.size(); ++i)
    {
        for (int j = 0; j < segmentsPerEdge; ++j)
        {
            if (stopSignal.load()) return;

            float t = static_cast<float>(j) / segmentsPerEdge;

            transmuter::geometry::Vertex v;
            // Lerp position between corner A and corner B
            v.pos = corners[i] * (1.0f - t) + corners[i + 1] * t;

            // Keep the tool normal to the face we are currently on
            v.normal = faceNormals[i];

            p_toolPath->addVertex(v);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    INFO("Edge-tracing path generation complete.");
}

}  // namespace transmuter::pathgenerator::algorithms