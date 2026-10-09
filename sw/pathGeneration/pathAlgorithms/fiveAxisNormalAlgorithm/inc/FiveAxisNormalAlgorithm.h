#pragma once
#include "PathAlgorithmIf.h"

namespace transmuter::pathgenerator::algorithms
{
/** Experimental stock-surface peeling with collision and preload constrained candidate selection. */
class FiveAxisNormalAlgorithm final : public PathAlgorithmIf
{
   public:
    FiveAxisNormalAlgorithm(const geometry::MillingScene& scene, PathAlgorithmConfig config);
    PathGenerationResult generate(const std::atomic<bool>& stopRequested) override;

   private:
    PathAlgorithmConfig m_config;
};
}  // namespace transmuter::pathgenerator::algorithms
