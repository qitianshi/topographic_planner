#include "smooth.hpp"
#include <cassert>

namespace smooth {

/* Interpolate two points by a factor 'c' */
static terrainmap::Point2D lerp(const terrainmap::Point2D& a, const terrainmap::Point2D& b, float c) {
    if (c < 0.0f) c = 0.0f;
    if (c > 1.0f) c = 1.0f;
    return terrainmap::Point2D{
        a.x * c + b.x * (1.0f - c),
        a.y * c + b.y * (1.0f - c)
    };
}

/* Calculate the cost at a particular point in the map */
static uint8_t getCost(const terrainmap::TerrainMap& map, const terrainmap::Point2D& point) {
    const uint32_t index = map.getPos(point);
    assert(index < map.npointsX * map.npointsY);
    return map.dataCost[index];
}

/* A single smooth step over a path, respecting the cost map */
void step(const terrainmap::TerrainMap& map, std::vector<terrainmap::Point2D>& path) {
    if (path.size() < 2) return;

    terrainmap::Point2D first = path[0];
    terrainmap::Point2D* second = &path[1];

    for (size_t i = 1; i < path.size(); ++i) {
        terrainmap::Point2D* third = &path[i];
        const terrainmap::Point2D base = lerp(first, *third, 0.5f); // Midpoint of first and third

        float factor = 0.5f - static_cast<float>(getCost(map, *second) - getCost(map, base)) / 10.0f;
        const terrainmap::Point2D newPoint = lerp(*second, base, factor);

        first = *second; // Second moves to first
        *second = newPoint; // The actual path changes
        second = third; // Third moves to second
    }
}

/* Smooth an entire path using multiple steps */
void apply(const terrainmap::TerrainMap& map, uint32_t steps, std::vector<terrainmap::Point2D>& path) {
    for (uint32_t i = 0; i < steps; ++i) {
        step(map, path);
    }
}

}
