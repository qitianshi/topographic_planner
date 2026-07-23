#include "topographic_planner/smooth.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

namespace smooth {

/* Interpolate two points by a factor 'c' */
static grid_map::Position
lerp(const grid_map::Position &a, const grid_map::Position &b, float c) {
    if (c < 0.0f)
        c = 0.0f;
    if (c > 1.0f)
        c = 1.0f;
    return a * c + b * (1.0f - c);
}

/* Calculate the cost at a particular point in the map */
static float
getCost(const grid_map::GridMap &map, const grid_map::Position &point) {
    if (!map.isInside(point)) {
        return 255.0f; // High cost outside
    }
    return map.atPosition("traversability", point);
}

/* A single smooth step over a path, respecting the cost map */
void step(const grid_map::GridMap &map, std::vector<grid_map::Position> &path) {
    if (path.size() < 2)
        return;

    grid_map::Position first = path[0];
    grid_map::Position *second = &path[1];

    for (size_t i = 1; i < path.size(); ++i) {
        grid_map::Position *third = &path[i];
        const grid_map::Position base =
            lerp(first, *third, 0.5f); // Midpoint of first and third

        float factor =
            0.5f - (getCost(map, *second) - getCost(map, base)) / 10.0f;
        const grid_map::Position newPoint = lerp(*second, base, factor);

        first = *second;    // Second moves to first
        *second = newPoint; // The actual path changes
        second = third;     // Third moves to second
    }
}

/* Smooth an entire path using multiple steps */
void apply(
    const grid_map::GridMap &map,
    uint32_t steps,
    std::vector<grid_map::Position> &path
) {
    for (uint32_t i = 0; i < steps; ++i) {
        step(map, path);
    }
}

} // namespace smooth
