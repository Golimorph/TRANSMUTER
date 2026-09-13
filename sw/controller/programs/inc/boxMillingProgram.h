#ifndef TRANSMUTER_CONTROLLER_PROGRAMS_BOX_MILLING_H
#define TRANSMUTER_CONTROLLER_PROGRAMS_BOX_MILLING_H

#include <memory>

#include "geometryIf.h"
#include "millingScene.h"
#include "pathGenerator.h"
#include "pointCloud.h"
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
    // Scene solids use the representation-independent geometry boundary.
    transmuter::geometry::MillingScene createMillingScene();
    std::shared_ptr<const transmuter::geometry::GeometryIf> m_stock;
    std::shared_ptr<const transmuter::geometry::GeometryIf> m_model;
    std::shared_ptr<transmuter::geometry::GeometryIf> m_workpiece;
    std::shared_ptr<transmuter::geometry::PointCloud> m_toolPath;
    std::shared_ptr<const transmuter::geometry::GeometryIf> m_toolModel;

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
