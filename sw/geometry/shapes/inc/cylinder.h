#ifndef TRANSMUTER_GEOMETRY_SHAPES_CYLINDER_H
#define TRANSMUTER_GEOMETRY_SHAPES_CYLINDER_H

#include <Eigen/Dense>
#include <memory>

#include "analyticShape.h"
#include "pointCloud.h"
#include "vertex.h"

namespace transmuter::geometry::shapes
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

    /**
     * @brief Generates the sampled boundary of a cylinder coaxial with +Z.
     * @param radius Cylinder radius in the XY plane.
     * @param height Cylinder extent along Z.
     * @param density The distance between points on the surface (mm).
     * @return A PointCloud object populated with Vertices and Normals.
     */
    static std::shared_ptr<PointCloud> generate(double radius, double height, double density);

    /**
     * @brief Generates points throughout the volume of a cylinder coaxial with +Z.
     * @param density Approximate spacing between volume samples (mm).
     */
    static std::shared_ptr<PointCloud> generateSolid(double radius, double height, double density);

   private:
    double m_radius;
    double m_height;
};

}  // namespace transmuter::geometry::shapes

#endif
