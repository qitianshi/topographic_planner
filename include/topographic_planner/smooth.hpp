#pragma once

#include <grid_map_core/grid_map_core.hpp>

#include "topographic_planner/pathfinder.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace smooth {

void apply(
    const grid_map::GridMap &map,
    const std::string &trav_layer,
    uint32_t steps,
    pathfinder::Path &path
);

}    // namespace smooth
