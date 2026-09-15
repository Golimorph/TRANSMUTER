#ifndef TRANSMUTER_GEOMETRY_ANALYTIC_SHAPES_CYLINDER_H
#define TRANSMUTER_GEOMETRY_ANALYTIC_SHAPES_CYLINDER_H

#include <Eigen/Dense>
#include <memory>

#include "analyticShape.h"

namespace transmuter::geometry::analyticshapes
{

/** Analytic cylinder along local +Z, with its base at z = 0. */
class Cylinder final : public AnalyticShape
{
   public:
    Cylinder(double radius, double height);

    bool isInside(const Eigen::Vector3d& point) const override;
    GeometryBounds bounds() const override;
    double maximumDistanceFromAxis(const Eigen::Vector3d& axisPoint, const Eigen::Vector3d& axisDirection) const override;
    std::shared_ptr<GeometryIf> clone() const override;
    void copyVisualizationVertices(std::vector<Vertex>& output) const override;

   private:
    double m_radius;
    double m_height;
};

}  // namespace transmuter::geometry::analyticshapes

#endif  // TRANSMUTER_GEOMETRY_ANALYTIC_SHAPES_CYLINDER_H
