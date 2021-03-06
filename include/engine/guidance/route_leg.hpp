#ifndef ROUTE_LEG_HPP
#define ROUTE_LEG_HPP

#include "engine/guidance/route_step.hpp"

#include <boost/optional.hpp>

#include <string>
#include <vector>

namespace osrm
{
namespace engine
{
namespace guidance
{

struct RouteLeg
{
    double duration;
    double distance;
    std::string summary;
    std::vector<RouteStep> steps;
};
}
}
}

#endif
