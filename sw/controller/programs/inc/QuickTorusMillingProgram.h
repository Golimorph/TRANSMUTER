#pragma once
#include <memory>

#include "PathGenerator.h"
#include "Visualizer.h"
#include "programIf.h"

namespace transmuter::controller::programs
{
class QuickTorusMillingProgram final : public ProgramIf
{
   public:
    void execute() override;

   private:
    geometry::MillingScene createMillingScene() const;
    pathgenerator::PathAlgorithmConfig fineCuttingConfig() const;
    pathgenerator::PathAlgorithmConfig roughingConfig() const;
    void startCuttingOperation(pathgenerator::PathAlgorithmType algorithm, geometry::MillingScene scene, const pathgenerator::PathAlgorithmConfig& config, visualization::Visualizer& visualizer);
    void appendOperationToCombinedPath(const pathgenerator::PathGenerationResult& result, std::shared_ptr<const geometry::GeometryIf> currentMaterial, double geometrySampleSpacing);
    geometry::MillingScene m_scene;
    std::shared_ptr<pathgenerator::MachinePathIf> m_combinedPath;
    std::unique_ptr<pathgenerator::PathGenerator> m_pathGenerator;
};
}  // namespace transmuter::controller::programs
