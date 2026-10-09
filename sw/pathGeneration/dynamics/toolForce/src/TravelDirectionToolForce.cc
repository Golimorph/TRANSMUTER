#include "TravelDirectionToolForce.h"

#include <cmath>

namespace transmuter::pathgenerator
{
Eigen::Vector3d TravelDirectionToolForce::forceOnWorkpiece(const geometry::Vertex& from, const geometry::Vertex& to) const
{
    const Eigen::Vector3d travel = (to.pos - from.pos).normalized();
    const Eigen::Vector3d sideways = travel.cross(to.normal.normalized());
    if (sideways.squaredNorm() < 1e-20)
    {
        return travel;
    }

    // The workpiece receives the reaction to the opposing and sideways loads on the cutter.
    return (travel - sideways.normalized()) / std::sqrt(2.0);
}

}  // namespace transmuter::pathgenerator
