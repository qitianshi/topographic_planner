#include "terrainmap.hpp"
#include <cstdint>
#include <cstdlib>

namespace terrainmap {

bool TerrainMap::alloc() {
    size_t numElements =
        static_cast<size_t>(npointsX) * static_cast<size_t>(npointsY);

    if (numElements == 0) {
        dataElevation.clear();
        dataCost.clear();
        return false;
    }

    try {
        dataElevation = std::vector<PhysDist>(numElements);
        dataCost = std::vector<uint8_t>(numElements);
    } catch (...) {
        dataElevation.clear();
        dataCost.clear();
        return false;
    }

    return true;
}

void TerrainMap::free() {
    dataElevation.clear();
    dataElevation.shrink_to_fit();
    dataCost.clear();
    dataCost.shrink_to_fit();
}

uint32_t TerrainMap::getNpoints() const {
    return npointsX * npointsY;
}

bool TerrainMap::contains(uint32_t pos) const {
    return pos < getNpoints();
}

uint32_t TerrainMap::getPos(const Point2D& p) const {
    const uint32_t cellX = static_cast<uint32_t>(p.x / resolution);
    const uint32_t cellY = static_cast<uint32_t>(p.y / resolution);
    return cellY * npointsX + cellX;
}

GridCoord TerrainMap::getCoord(uint32_t pos) const {
    if (npointsX == 0) return {0, 0};
    return GridCoord{
        pos % npointsX, // x
        pos / npointsX  // y
    };
}

Point2D TerrainMap::toPoint(const GridCoord& coord) const {
    return Point2D{
        static_cast<PhysDist>(coord.x) * resolution,
        static_cast<PhysDist>(coord.y) * resolution
    };
}

bool TerrainMap::isDiagonal(uint32_t curPos, uint32_t nextPos) const {
    GridCoord curCoord = getCoord(curPos);
    GridCoord nextCoord = getCoord(nextPos);
    return curCoord.x != nextCoord.x && curCoord.y != nextCoord.y;
}

}
