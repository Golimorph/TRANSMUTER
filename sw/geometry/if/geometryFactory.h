#ifndef TRANSMUTER_GEOMETRY_GEOMETRY_FACTORY_H
#define TRANSMUTER_GEOMETRY_GEOMETRY_FACTORY_H

#include <memory>
#include <optional>

#include "geometryIf.h"

namespace transmuter::geometry
{

enum class GeometryRepresentation
{
    ANALYTIC,
    POINT_CLOUD_SOLID,
    POINT_CLOUD_SURFACE
};

/** Public construction boundary for geometry implementations. */
class GeometryFactory
{
   public:
    GeometryFactory() = delete;

    static std::shared_ptr<GeometryIf> createBox(double sizeX, double sizeY, double sizeZ, GeometryRepresentation representation, double sampleSpacing);
    static std::shared_ptr<GeometryIf> createCylinder(double radius, double height, GeometryRepresentation representation, std::optional<double> sampleSpacing = std::nullopt);
    static std::shared_ptr<GeometryIf> createDodecahedron(double circumradius, GeometryRepresentation representation, double sampleSpacing);
    static std::shared_ptr<GeometryIf> createTorus(double majorRadius, double tubeRadius, GeometryRepresentation representation, double sampleSpacing);
};

}  // namespace transmuter::geometry

#endif  // TRANSMUTER_GEOMETRY_GEOMETRY_FACTORY_H
