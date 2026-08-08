#include "topographic_planner/smooth.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace smooth {

/** Interpolate two points by a factor 'c' */
static grid_map::Position lerp(
    const grid_map::Position &a,
    const grid_map::Position &b,
    float c
) {
    if (c < 0.0f)
        c = 0.0f;
    if (c > 1.0f)
        c = 1.0f;
    return a * c + b * (1.0f - c);
}

/** Calculate the cost at a particular point in the map */
static float getCost(
    const grid_map::GridMap &map,
    const std::string &layer,
    const grid_map::Position &point
) {
    if (!map.isInside(point)) {
        return 255.0f;    // High cost outside
    }
    return map.atPosition(layer, point);
}

/** A single smooth step over a path, respecting the cost map */
void step(
    const grid_map::GridMap &map,
    const std::string &layer,
    pathfinder::Path &path
) {
    if (path.size() < 3)
        return;

    grid_map::Position prev = path[0];
    for (size_t i = 1; i < path.size() - 1; ++i) {
        grid_map::Position curr = path[i];
        grid_map::Position next = path[i + 1];

        const grid_map::Position base = lerp(prev, next, 0.5f);
        float factor =
            0.5f -
            (getCost(map, layer, curr) - getCost(map, layer, base)) / 10.0f;

        path[i] = lerp(curr, base, factor);
        prev = curr;    // Use original point for next iteration's base
    }
}

/** Smooth an entire path using multiple steps */
void apply(
    const grid_map::GridMap &map,
    const std::string &trav_layer,
    uint32_t steps,
    pathfinder::Path &path
) {
    for (uint32_t i = 0; i < steps; ++i) {
        step(map, trav_layer, path);
    }
}

}    // namespace smooth
