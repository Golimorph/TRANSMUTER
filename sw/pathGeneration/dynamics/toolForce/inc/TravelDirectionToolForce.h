#pragma once
#include "ToolForceIf.h"

namespace transmuter::pathgenerator
{
class TravelDirectionToolForce final : public ToolForceIf
{
   public:
    Eigen::Vector3d forceOnWorkpiece(const geometry::Vertex& from, const geometry::Vertex& to) const override;
};
}  // namespace transmuter::pathgenerator
