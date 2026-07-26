#pragma once

#include <grid_map_core/grid_map_core.hpp>

#include <cstdint>
#include <vector>

namespace smooth {

void apply(
    const grid_map::GridMap &map,
    uint32_t steps,
    std::vector<grid_map::Position> &path
);

} // namespace smooth
