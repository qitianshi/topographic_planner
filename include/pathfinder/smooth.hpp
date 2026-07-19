#ifndef SMOOTH_HPP
#define SMOOTH_HPP

#include "map.hpp"
#include <vector>
#include <cstdint>

#define SMOOTH_STEPS_DEFAULT 10

namespace smooth {

void step(const terrainmap::TerrainMap& map, std::vector<terrainmap::Point2D>& path);

void apply(const terrainmap::TerrainMap& map, uint32_t steps, std::vector<terrainmap::Point2D>& path);

} // namespace smooth

#endif // SMOOTH_HPP
