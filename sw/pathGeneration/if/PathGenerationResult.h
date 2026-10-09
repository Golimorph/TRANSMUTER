#ifndef TRANSMUTER_PATH_GENERATOR_PATH_GENERATION_RESULT_H
#define TRANSMUTER_PATH_GENERATOR_PATH_GENERATION_RESULT_H

#include <memory>
#include <string>

#include "MachinePathIf.h"
#include "geometryIf.h"

namespace transmuter::pathgenerator
{

/**
 * The artifacts produced by one local machining operation.
 *
 * The generator snapshots machinePath at completion; live preview reads the machine's path.
 * workpiece stays private during generation and is returned as immutable stage output.
 * Every published move has been accepted by machine kinematics.
 */
struct PathGenerationResult
{
    std::shared_ptr<const MachinePathIf> machinePath;
    std::shared_ptr<const geometry::GeometryIf> workpiece;
    std::string generationNote;
};

}  // namespace transmuter::pathgenerator

#endif  // TRANSMUTER_PATH_GENERATOR_PATH_GENERATION_RESULT_H
