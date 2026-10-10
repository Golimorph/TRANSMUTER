#pragma once
#include <memory>
#include <optional>

#include "MillingProgramContext.h"
#include "PathGenerator.h"
#include "Visualizer.h"
#include "programIf.h"

namespace transmuter::controller::programs
{
class InteractiveTorusMillingProgram final : public ProgramIf
{
   public:
    void execute() override;

   private:
    void initializeSceneAndVisualizer();
    std::optional<visualization::UserMillingRequest> receiveUserMillingRequest();
    void startSelectedOperation(const visualization::UserMillingRequest& request);
    void finishSelectedOperation();
    void stopPathGeneration();
    MillingProgramContext createMillingContext() const;

    std::optional<MillingProgramContext> m_programContext;
    std::unique_ptr<pathgenerator::PathGenerator> m_pathGenerator;
    std::unique_ptr<visualization::Visualizer> m_visualizer;
};
}  // namespace transmuter::controller::programs
