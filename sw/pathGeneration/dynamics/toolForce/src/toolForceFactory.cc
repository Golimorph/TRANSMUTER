#include "toolForceFactory.h"

#include "TravelDirectionToolForce.h"

namespace transmuter::pathgenerator
{
std::shared_ptr<const ToolForceIf> travelDirectionToolForce()
{
    return std::make_shared<TravelDirectionToolForce>();
}
}  // namespace transmuter::pathgenerator
