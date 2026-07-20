#pragma once

#include <cstdint>
#include <vector>

namespace terrainmap {

/* Denotes that this value represents a physical distance */
using PhysDist = float;

struct Point2D {
    PhysDist x;
    PhysDist y;
};

struct GridCoord {
    uint32_t x;
    uint32_t y;
};

/* Map of the terrain to navigate, containing multiple data layers */
struct TerrainMap {
    // Metadata
    uint32_t npointsX = 0;      // Number of data points along x-axis
    uint32_t npointsY = 0;      // Number of data points along y-axis
    PhysDist resolution = 0.0f; // Physical distance (e.g., meters) per point

    // Data arrays
    // All array lengths are (npointsX * npointsY).
    std::vector<PhysDist> dataElevation; // Height along z-axis
    std::vector<uint8_t> dataCost;       // General cost

    TerrainMap() = default;
    ~TerrainMap() { free(); }

    // Allocates the data arrays for `TerrainMap`
    // The metadata fields must already be populated.
    bool alloc();

    // Clears the data arrays for `TerrainMap`
    void free();

    uint32_t getNpoints() const;

    bool contains(uint32_t pos) const;

    uint32_t getPos(const Point2D &p) const;

    GridCoord getCoord(uint32_t pos) const;

    Point2D toPoint(const GridCoord &coord) const;

    bool isDiagonal(uint32_t curPos, uint32_t nextPos) const;
};

} // namespace terrainmap
