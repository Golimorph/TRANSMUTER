#ifndef TRANSMUTER_GEOMETRY_GEOMETRY_FACTORY_H
#define TRANSMUTER_GEOMETRY_GEOMETRY_FACTORY_H

#include <Eigen/Dense>
#include <memory>

#include "geometryIf.h"
#include "toolPathIf.h"

namespace transmuter::geometry
{

enum class ShapeSampling
{
    SURFACE,
    SOLID
};

/** Public construction boundary for geometry and tool-path implementations. */
class GeometryFactory
{
   public:
    GeometryFactory() = delete;

    static std::shared_ptr<GeometryIf> createBox(
        const Eigen::Vector3d& dimensions, double sampleSpacing, ShapeSampling sampling);
    static std::shared_ptr<GeometryIf> createCylinder(
        double radius, double height, double sampleSpacing, ShapeSampling sampling);
    static std::shared_ptr<GeometryIf> createDodecahedron(
        double circumradius, double sampleSpacing, ShapeSampling sampling);
    static std::shared_ptr<GeometryIf> createTorus(
        double majorRadius, double tubeRadius, double sampleSpacing, ShapeSampling sampling);
    static std::shared_ptr<GeometryIf> createEndmill(double diameter, double length);
    static std::shared_ptr<ToolPathIf> createToolPath();
};

}  // namespace transmuter::geometry

#endif  // TRANSMUTER_GEOMETRY_GEOMETRY_FACTORY_H
