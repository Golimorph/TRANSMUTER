#ifndef TRANSMUTER_PATH_GENERATOR_PATHGENERATOR_H
#define TRANSMUTER_PATH_GENERATOR_PATHGENERATOR_H

#include <atomic>
#include <memory>
#include <thread>

#include "millingScene.h"
#include "pathAlgorithmIf.h"
#include "pathGenerationResult.h"

namespace transmuter::pathgenerator
{

class PathGenerator
{
   public:
    /** Selects the concrete implementation constructed by PathGenerator. */
    enum class AlgorithmType
    {
        THREE_AXIS_SPIRAL
    };

    PathGenerator(AlgorithmType algorithmType, PathGeneratorConfig config, const transmuter::geometry::MillingScene& millingScene);
    ~PathGenerator();
    void startGeneration();
    void stopGeneration();
    bool isGenerationFinished() const;
    bool isGenerationInProgress() const;
    PathGenerationResult getResult() const;

   private:
    std::unique_ptr<transmuter::pathgenerator::algorithms::PathAlgorithmIf> m_algorithm;
    std::thread m_thread;
    std::atomic<bool> m_stopRequested{false};
    std::atomic<bool> m_isGenerating{false};
    std::shared_ptr<transmuter::geometry::ToolPathIf> m_pathOutput;
    PathGenerationResult m_result;
    std::atomic<bool> m_isResultReady{false};

    void launchGenerationThread();
};

}  // namespace transmuter::pathgenerator

#endif  // TRANSMUTER_PATH_GENERATOR_PATHGENERATOR_H
