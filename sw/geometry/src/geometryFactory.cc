#include "geometryFactory.h"

#include "cylinder.h"
#include "boxGenerator.h"
#include "cylinderGenerator.h"
#include "dodecahedronGenerator.h"
#include "torusGenerator.h"
#include "toolPath.h"
#include "trace.h"

namespace transmuter::geometry
{

namespace
{

bool usesSurfacePointCloud(GeometryRepresentation representation)
{
    switch (representation)
    {
        case GeometryRepresentation::POINT_CLOUD_SURFACE:
            return true;
        case GeometryRepresentation::POINT_CLOUD_SOLID:
            return false;
        case GeometryRepresentation::ANALYTIC:
            ERROR("This geometry is not available as an analytic representation.");
        default:
            ERROR("Unknown geometry representation.");
    }
    return false;
}

std::shared_ptr<GeometryIf> createPointCloudCylinder(double radius, double height, double sampleSpacing, GeometryRepresentation representation)
{
    const bool surfaceSampling = usesSurfacePointCloud(representation);
    INFO("Creating " << (surfaceSampling ? "surface" : "solid") << " point-cloud cylinder: radius "
                      << radius << " mm, height " << height << " mm, sample spacing " << sampleSpacing
                      << " mm, centered at the origin with its axis along +Z.");
    if (surfaceSampling)
    {
        return pointcloudshapes::CylinderGenerator::generate(radius, height, sampleSpacing);
    }
    return pointcloudshapes::CylinderGenerator::generateSolid(radius, height, sampleSpacing);
}

}  // namespace

std::shared_ptr<GeometryIf> GeometryFactory::createBox(double sizeX, double sizeY, double sizeZ, GeometryRepresentation representation, double sampleSpacing)
{
    const Eigen::Vector3d dimensions(sizeX, sizeY, sizeZ);
    const bool surfaceSampling = usesSurfacePointCloud(representation);
    INFO("Creating " << (surfaceSampling ? "surface" : "solid") << " box: " << dimensions.x()
                      << " x " << dimensions.y() << " x " << dimensions.z() << " mm, sample spacing "
                      << sampleSpacing << " mm, centered at the origin and aligned with the XYZ axes.");
    if (surfaceSampling)
    {
        return pointcloudshapes::BoxGenerator::generate(dimensions, sampleSpacing);
    }
    return pointcloudshapes::BoxGenerator::generateSolid(dimensions, sampleSpacing);
}

std::shared_ptr<GeometryIf> GeometryFactory::createCylinder(double radius, double height, GeometryRepresentation representation, std::optional<double> sampleSpacing)
{
    switch (representation)
    {
        case GeometryRepresentation::ANALYTIC:
            INFO("Creating analytic cylinder: radius " << radius << " mm, height " << height
                                                        << " mm, with its base-center origin and axis along +Z.");
            return std::make_shared<analyticshapes::Cylinder>(radius, height);
        case GeometryRepresentation::POINT_CLOUD_SOLID:
        case GeometryRepresentation::POINT_CLOUD_SURFACE:
            if (!sampleSpacing.has_value())
            {
                ERROR("Point-cloud cylinder sample spacing must be specified in mm.");
            }
            return createPointCloudCylinder(radius, height, *sampleSpacing, representation);
        default:
            ERROR("Unknown geometry representation.");
    }
    return nullptr;
}

std::shared_ptr<GeometryIf> GeometryFactory::createDodecahedron(double circumradius, GeometryRepresentation representation, double sampleSpacing)
{
    if (usesSurfacePointCloud(representation))
    {
        return pointcloudshapes::DodecahedronGenerator::generate(circumradius, sampleSpacing);
    }
    return pointcloudshapes::DodecahedronGenerator::generateSolid(circumradius, sampleSpacing);
}

std::shared_ptr<GeometryIf> GeometryFactory::createTorus(double majorRadius, double tubeRadius, GeometryRepresentation representation, double sampleSpacing)
{
    if (usesSurfacePointCloud(representation))
    {
        return pointcloudshapes::TorusGenerator::generate(majorRadius, tubeRadius, sampleSpacing);
    }
    return pointcloudshapes::TorusGenerator::generateSolid(majorRadius, tubeRadius, sampleSpacing);
}

std::shared_ptr<ToolPathIf> GeometryFactory::createToolPath()
{
    INFO("Creating an empty tool path.");
    return std::make_shared<ToolPath>();
}

}  // namespace transmuter::geometry
