#ifndef TRANSMUTER_CONTROLLER_PROGRAMS_BOX_MILLING_H
#define TRANSMUTER_CONTROLLER_PROGRAMS_BOX_MILLING_H

#include <memory>

#include "millingScene.h"
#include "pathGenerator.h"
#include "programIf.h"
#include "visualizer.h"

namespace transmuter::controller::programs
{

/**
 * @brief The BoxMillingProgram is a simplistic demonstration of how to mill a cylinder into a cube
 * It sets up the environment, performs the path calculation, and then does a material removal
 * simulation along the path.
 *  */
class BoxMillingProgram : public ProgramIf
{
   public:
    BoxMillingProgram() = default;
    ~BoxMillingProgram() override = default;

    void execute() override;

   private:
    // Milling scene variables are shared so generation and visualization observe the same state.
    transmuter::geometry::MillingScene createMillingScene();
    std::shared_ptr<transmuter::geometry::PointCloud>
        m_stock;  // The original shape before cutting starts.
    std::shared_ptr<transmuter::geometry::PointCloud>
        m_model;  // The desired model I want as end result.
    std::shared_ptr<transmuter::geometry::PointCloud>
        m_workpiece;  // The actual state of the model during milling operation visualization.
    std::shared_ptr<transmuter::geometry::PointCloud> m_toolPath;  // The path the tool will take.
    std::shared_ptr<transmuter::geometry::PointCloud>
        m_toolModel;  // The model of the tool itself. For example a 3mm end mill would be a
                      // cylinder.

    struct MillingEnvironment
    {
        transmuter::visualization::Visualizer visualizer;
        transmuter::pathgenerator::PathGenerator pathGenerator;
    };

    MillingEnvironment createMillingEnvironment(
        const transmuter::geometry::MillingScene& millingScene) const;
};

}  // namespace transmuter::controller::programs
#endif
