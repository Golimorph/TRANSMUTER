#include "../if/geometryFactory.h"

#include "../pointCloud/inc/pointCloud.h"
#include "../shapes/analyticShapes/inc/endmill.h"
#include "../shapes/pointCloudShapes/inc/boxGenerator.h"
#include "../shapes/pointCloudShapes/inc/cylinderGenerator.h"
#include "../shapes/pointCloudShapes/inc/dodecahedronGenerator.h"
#include "../shapes/pointCloudShapes/inc/torusGenerator.h"
#include "trace.h"

namespace transmuter::geometry
{

namespace
{

bool usesSurfaceSampling(ShapeSampling sampling)
{
    switch (sampling)
    {
        case ShapeSampling::SURFACE:
            return true;
        case ShapeSampling::SOLID:
            return false;
        default:
            ERROR("Unknown shape sampling mode.");
    }
    return false;
}

}  // namespace

std::shared_ptr<GeometryIf> GeometryFactory::createBox(
    const Eigen::Vector3d& dimensions, double sampleSpacing, ShapeSampling sampling)
{
    if (usesSurfaceSampling(sampling))
    {
        return pointcloudshapes::BoxGenerator::generate(dimensions, sampleSpacing);
    }
    return pointcloudshapes::BoxGenerator::generateSolid(dimensions, sampleSpacing);
}

std::shared_ptr<GeometryIf> GeometryFactory::createCylinder(
    double radius, double height, double sampleSpacing, ShapeSampling sampling)
{
    if (usesSurfaceSampling(sampling))
    {
        return pointcloudshapes::CylinderGenerator::generate(radius, height, sampleSpacing);
    }
    return pointcloudshapes::CylinderGenerator::generateSolid(radius, height, sampleSpacing);
}

std::shared_ptr<GeometryIf> GeometryFactory::createDodecahedron(
    double circumradius, double sampleSpacing, ShapeSampling sampling)
{
    if (usesSurfaceSampling(sampling))
    {
        return pointcloudshapes::DodecahedronGenerator::generate(circumradius, sampleSpacing);
    }
    return pointcloudshapes::DodecahedronGenerator::generateSolid(circumradius, sampleSpacing);
}

std::shared_ptr<GeometryIf> GeometryFactory::createTorus(
    double majorRadius, double tubeRadius, double sampleSpacing, ShapeSampling sampling)
{
    if (usesSurfaceSampling(sampling))
    {
        return pointcloudshapes::TorusGenerator::generate(majorRadius, tubeRadius, sampleSpacing);
    }
    return pointcloudshapes::TorusGenerator::generateSolid(majorRadius, tubeRadius, sampleSpacing);
}

std::shared_ptr<GeometryIf> GeometryFactory::createEndmill(double diameter, double length)
{
    return analyticshapes::Endmill::generate(diameter, length);
}

std::shared_ptr<ToolPathIf> GeometryFactory::createToolPath()
{
    return std::make_shared<PointCloud>();
}

}  // namespace transmuter::geometry
