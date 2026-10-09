#ifndef TRANSMUTER_GEOMETRY_MILLING_SCENE_H
#define TRANSMUTER_GEOMETRY_MILLING_SCENE_H

#include <memory>

#include "geometryIf.h"

namespace transmuter::pathgenerator
{
class MachineKinematicsIf;
class MachinePathIf;
}  // namespace transmuter::pathgenerator

namespace transmuter::geometry
{

struct MillingScene
{
    // Immutable shape inputs for this scene. A local stock may be an extracted, already-machined
    // section of a larger workpiece.
    std::shared_ptr<const GeometryIf> stock;
    std::shared_ptr<const GeometryIf> model;

    // Shared live state used by execution and visualization.
    std::shared_ptr<GeometryIf> workpiece;
    std::shared_ptr<const pathgenerator::MachinePathIf> machinePath;

    // The tool geometry is also an immutable input to the operation.
    std::shared_ptr<const GeometryIf> toolModel;
    // Optional full cutter/shank envelope in the same tool frame; only toolModel removes stock.
    std::shared_ptr<const GeometryIf> toolAssemblyModel;

    // One physical machine is shared by every cutting stage and by playback.
    std::shared_ptr<pathgenerator::MachineKinematicsIf> machineKinematics;
};

}  // namespace transmuter::geometry
#endif  // TRANSMUTER_GEOMETRY_MILLING_SCENE_H
