#pragma once
#include <Eigen/Geometry>

#include "vertex.h"

namespace transmuter::geometry
{
inline Eigen::Affine3d toolTransform(const Vertex& pose)
{
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), pose.normal.normalized())
                             .toRotationMatrix();
    transform.translation() = pose.pos;
    return transform;
}
inline Vertex interpolatePose(const Vertex& from, const Vertex& to, double fraction)
{
    const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(from.normal.normalized(),
        to.normal.normalized());
    return {from.pos + fraction * (to.pos - from.pos),
        Eigen::Quaterniond::Identity().slerp(fraction, rotation) * from.normal.normalized()};
}
}  // namespace transmuter::geometry
