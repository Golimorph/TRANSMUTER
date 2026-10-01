#ifndef TRANSMUTER_GEOMETRY_ANALYTIC_SHAPES_ANALYTIC_SHAPE_H
#define TRANSMUTER_GEOMETRY_ANALYTIC_SHAPES_ANALYTIC_SHAPE_H

#include "geometryIf.h"

namespace transmuter::geometry::analyticshapes
{

/** Immutable analytic geometry described by a small set of parameters. */
class AnalyticShape : public GeometryIf
{
   public:
    bool intersects(const GeometryIf& geometry, const Eigen::Affine3d& geometryToLocalTransform = Eigen::Affine3d::Identity()) const override;
    void combine(const GeometryIf& geometry, const Eigen::Affine3d& geometryToLocalTransform = Eigen::Affine3d::Identity()) override;
    void cut(const GeometryIf& cuttingGeometry, const Eigen::Affine3d& cuttingToThis = Eigen::Affine3d::Identity()) override;
};

}  // namespace transmuter::geometry::analyticshapes

#endif  // TRANSMUTER_GEOMETRY_ANALYTIC_SHAPES_ANALYTIC_SHAPE_H
