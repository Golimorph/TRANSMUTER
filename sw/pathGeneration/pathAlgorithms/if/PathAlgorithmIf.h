#ifndef TRANSMUTER_PATHGEN_PATH_ALGORITHM_IF_H
#define TRANSMUTER_PATHGEN_PATH_ALGORITHM_IF_H

#include <atomic>
#include <memory>

#include "MachineKinematicsIf.h"
#include "PathAlgorithmConfig.h"
#include "PathAlgorithmType.h"
#include "PathGenerationResult.h"
#include "geometryIf.h"
#include "millingScene.h"

namespace transmuter::pathgenerator::algorithms
{

class PathAlgorithmIf
{
   public:
    PathAlgorithmIf(const geometry::MillingScene& millingScene, MachineKinematicsIf& machine);

    virtual ~PathAlgorithmIf() = default;

    // The machine records accepted moves; each algorithm owns its separate, mutable workpiece.
    virtual PathGenerationResult generate(const std::atomic<bool>& stopRequested) = 0;

   protected:
    const geometry::MillingScene& scene() const;
    const geometry::GeometryIf& operationStock() const;
    const geometry::GeometryIf& model() const;
    const geometry::GeometryIf& toolModel() const;
    geometry::GeometryIf& workpiece();
    std::shared_ptr<const geometry::GeometryIf> finishedWorkpiece() const;
    MachineKinematicsIf& machine();

   private:
    geometry::MillingScene m_scene;
    MachineKinematicsIf& m_machineKinematics;
    std::shared_ptr<geometry::GeometryIf> m_workpiece;
};

std::unique_ptr<PathAlgorithmIf> createPathAlgorithm(PathAlgorithmType type, const PathAlgorithmConfig& config, const geometry::MillingScene& scene, MachineKinematicsIf& machine);

}  // namespace transmuter::pathgenerator::algorithms

#endif  // TRANSMUTER_PATHGEN_PATH_ALGORITHM_IF_H
