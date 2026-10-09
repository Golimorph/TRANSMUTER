#include "analyticShape.h"

#include <algorithm>

#include "trace.h"

namespace transmuter::geometry::analyticshapes
{

bool AnalyticShape::intersects(const GeometryIf& geometry, const Eigen::Affine3d& geometryToLocalTransform) const
{
    std::vector<Vertex> geometrySamples;
    geometry.copyVisualizationVertices(geometrySamples);
    return std::any_of(geometrySamples.begin(), geometrySamples.end(),
        [&](const Vertex& sample) { return isInside(geometryToLocalTransform * sample.pos); });
}

void AnalyticShape::combine(const GeometryIf&, const Eigen::Affine3d&)
{
    ERROR("Analytic shapes cannot be modified by combining geometry.");
}

void AnalyticShape::cut(const GeometryIf&, const Eigen::Affine3d&)
{
    ERROR("Analytic shapes cannot be modified by cutting geometry.");
}

}  // namespace transmuter::geometry::analyticshapes
