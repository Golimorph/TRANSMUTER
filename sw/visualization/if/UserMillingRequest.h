#ifndef TRANSMUTER_VISUALIZATION_USER_MILLING_REQUEST_H
#define TRANSMUTER_VISUALIZATION_USER_MILLING_REQUEST_H

#include <Eigen/Geometry>
#include <memory>

#include "PathAlgorithmConfig.h"
#include "PathAlgorithmType.h"
#include "geometryIf.h"

namespace transmuter::visualization
{

struct UserMillingRequest
{
    std::shared_ptr<const geometry::GeometryIf> selectedVolume;
    Eigen::Affine3d volumeToWorkpiece = Eigen::Affine3d::Identity();
    pathgenerator::PathAlgorithmType algorithm;
    pathgenerator::PathAlgorithmConfig config;
};

}  // namespace transmuter::visualization

#endif  // TRANSMUTER_VISUALIZATION_USER_MILLING_REQUEST_H
