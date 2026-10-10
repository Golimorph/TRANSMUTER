#include "Box.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "boxGenerator.h"
#include "trace.h"

namespace transmuter::geometry::analyticshapes
{

Box::Box(const Eigen::Vector3d& dimensions) : m_dimensions(dimensions)
{
    if (!dimensions.allFinite() || (dimensions.array() <= 0.0).any())
    {
        ERROR("Analytic box dimensions must be finite and greater than zero.");
    }
}

bool Box::isInside(const Eigen::Vector3d& point) const
{
    return (point.cwiseAbs().array() <= (m_dimensions * 0.5).array()).all();
}

GeometryBounds Box::bounds() const
{
    const Eigen::Vector3d halfDimensions = m_dimensions * 0.5;
    return GeometryBounds{.min = -halfDimensions, .max = halfDimensions, .isValid = true};
}

double Box::maximumDistanceFromAxis(const Eigen::Vector3d& axisPoint, const Eigen::Vector3d& axisDirection) const
{
    const double axisLengthSquared = axisDirection.squaredNorm();
    if (!axisDirection.allFinite() || axisLengthSquared <= 0.0)
    {
        ERROR("Geometry axis direction must not be zero.");
    }

    const Eigen::Vector3d halfDimensions = m_dimensions * 0.5;
    double maximumDistanceSquared = 0.0;
    for (int x = -1; x <= 1; x += 2)
    {
        for (int y = -1; y <= 1; y += 2)
        {
            for (int z = -1; z <= 1; z += 2)
            {
                const Eigen::Vector3d offset(Eigen::Vector3d(x, y, z).cwiseProduct(halfDimensions) - axisPoint);
                const Eigen::Vector3d radialOffset = offset - axisDirection * (offset.dot(axisDirection) / axisLengthSquared);
                maximumDistanceSquared = std::max(maximumDistanceSquared, radialOffset.squaredNorm());
            }
        }
    }
    return std::sqrt(maximumDistanceSquared);
}

std::shared_ptr<GeometryIf> Box::clone() const
{
    return std::make_shared<Box>(m_dimensions);
}

void Box::copyVisualizationVertices(std::vector<Vertex>& output) const
{
    const double spacing = std::max(m_dimensions.minCoeff() / 10.0, 0.5);
    pointcloudshapes::BoxGenerator::generate(m_dimensions, spacing)->copyTo(output);
}

}  // namespace transmuter::geometry::analyticshapes
