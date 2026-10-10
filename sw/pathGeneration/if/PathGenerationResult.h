#ifndef TRANSMUTER_PATH_GENERATOR_PATH_GENERATION_RESULT_H
#define TRANSMUTER_PATH_GENERATOR_PATH_GENERATION_RESULT_H

#include <Eigen/Geometry>
#include <memory>
#include <string>

#include "MachinePathIf.h"
#include "geometryIf.h"

namespace transmuter::pathgenerator
{

/**
 * The output of one localized machining operation, ready to merge into program state.
 *
 * machinePath contains this operation's accepted moves. stock identifies the selected removal
 * region in workpiece coordinates, while workpiece contains the remaining local stock.
 */
struct PathGenerationResult
{
    std::shared_ptr<const MachinePathIf> machinePath;
    // Region removed from the program workpiece, expressed with this transform into workpiece coordinates.
    std::shared_ptr<const geometry::GeometryIf> stock;
    Eigen::Affine3d stockToWorkpiece = Eigen::Affine3d::Identity();
    std::shared_ptr<const geometry::GeometryIf> workpiece;
    std::string generationNote;
};

}  // namespace transmuter::pathgenerator

#endif  // TRANSMUTER_PATH_GENERATOR_PATH_GENERATION_RESULT_H
