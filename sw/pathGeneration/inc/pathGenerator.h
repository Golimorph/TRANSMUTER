#ifndef TRANSMUTER_PATH_GENERATOR_PATHGENERATOR_H
#define TRANSMUTER_PATH_GENERATOR_PATHGENERATOR_H

#include "millingScene.h"
#include "pathAlgorithmIf.h"
#include "pathGeneratorConfig.h"

namespace transmuter::pathgenerator
{

class PathGenerator
{
   public:
    PathGenerator(PathGeneratorConfig config,
                  const transmuter::geometry::MillingScene& millingScene);
    ~PathGenerator();
    void startGeneration();
    void stopGeneration();
    bool isGenerationFinished() const;
    bool isGenerationInProgress() const;

   private:
    std::unique_ptr<transmuter::pathgenerator::algorithms::PathAlgorithmIf> m_algorithm;
    std::thread m_thread;
    std::atomic<bool> m_stop{true};
    std::shared_ptr<transmuter::geometry::PointCloud> m_generatedToolPath;
    std::atomic<bool> m_isToolPathReady{false};

    void launchGenerationThread();
};

}  // namespace transmuter::pathgenerator

#endif  // TRANSMUTER_PATH_GENERATOR_PATHGENERATOR_H