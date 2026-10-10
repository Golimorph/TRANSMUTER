#ifndef TRANSMUTER_GEOMETRY_ANALYTIC_SHAPES_BOX_H
#define TRANSMUTER_GEOMETRY_ANALYTIC_SHAPES_BOX_H

#include <Eigen/Dense>

#include "analyticShape.h"

namespace transmuter::geometry::analyticshapes
{

/** Analytic axis-aligned box centered at the origin. */
class Box final : public AnalyticShape
{
   public:
    explicit Box(const Eigen::Vector3d& dimensions);

    bool isInside(const Eigen::Vector3d& point) const override;
    GeometryBounds bounds() const override;
    double maximumDistanceFromAxis(const Eigen::Vector3d& axisPoint, const Eigen::Vector3d& axisDirection) const override;
    std::shared_ptr<GeometryIf> clone() const override;
    void copyVisualizationVertices(std::vector<Vertex>& output) const override;

   private:
    Eigen::Vector3d m_dimensions;
};

}  // namespace transmuter::geometry::analyticshapes

#endif  // TRANSMUTER_GEOMETRY_ANALYTIC_SHAPES_BOX_H
