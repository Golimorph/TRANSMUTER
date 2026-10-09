#ifndef TRANSMUTER_PATH_GENERATOR_PATHGENERATOR_H
#define TRANSMUTER_PATH_GENERATOR_PATHGENERATOR_H

#include <atomic>
#include <memory>
#include <thread>

#include "PathAlgorithmConfig.h"
#include "PathAlgorithmType.h"
#include "PathGenerationResult.h"
#include "millingScene.h"

namespace transmuter::pathgenerator
{
class MachineKinematicsIf;

namespace algorithms
{
class PathAlgorithmIf;
}

class PathGenerator
{
   public:
    PathGenerator(PathAlgorithmType algorithmType, PathAlgorithmConfig config, const geometry::MillingScene& millingScene, std::atomic<bool>& stopRequested);
    ~PathGenerator();
    void startGeneration();
    void stopGeneration();
    bool isGenerationFinished() const;
    PathGenerationResult getResult() const;

   private:
    std::unique_ptr<transmuter::pathgenerator::algorithms::PathAlgorithmIf> m_algorithm;
    std::shared_ptr<MachineKinematicsIf> m_machineKinematics;
    std::thread m_thread;
    std::atomic<bool>& m_stopRequested;
    PathGenerationResult m_result;
    std::atomic<bool> m_isResultReady{false};

    void launchGenerationThread();
    void generatePath();
};

}  // namespace transmuter::pathgenerator

#endif  // TRANSMUTER_PATH_GENERATOR_PATHGENERATOR_H
