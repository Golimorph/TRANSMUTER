#ifndef TRANSMUTER_CONTROLLER_PROGRAMS_BOX_PROGRAM_H
#define TRANSMUTER_CONTROLLER_PROGRAMS_BOX_PROGRAM_H

#include <memory>

#include "pointCloud.h"
#include "programIf.h"

namespace transmuter::controller::programs
{

class BoxProgram : public ProgramIf
{
   public:
    BoxProgram() = default;
    ~BoxProgram() override = default;

    // The logic that runs in the background thread
    void execute() override;

   private:
    // This is where we will store the workpiece once generated
    geometry::PointCloud m_workPiece;
    float m_rotationAngle{0.0f};
};

}  // namespace transmuter::controller::programs
#endif