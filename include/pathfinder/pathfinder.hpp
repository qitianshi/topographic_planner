#pragma once

#include "terrainmap.hpp"
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace pathfinder {

struct ExtraOut {
    FILE *pathEval = nullptr;
    FILE *searchDisplay = nullptr;
    FILE *visual = nullptr; // visualize search
};

struct Weights {
    float wxyDist;
    float wgrad;
    float wcost;
    float wheuristic;
    float maxGrad;
};

struct Config {
    ExtraOut extraOut;
    Weights weights;
    terrainmap::Point2D start;
    terrainmap::Point2D dest;
};

enum class Result {
    FoundPath, // found a path from start to destination
    NoPath,    // could not find a path from start to destination
    ErrMem     // failed to find a path due to a memory allocation error
};

enum class VisualUpdateType : unsigned char {
    Tree,
    Queue,
    ChangeEdge,
    Finished
};

struct VisualUpdate {
    uint32_t pos;
    uint32_t parent;
    uint32_t oldParent;
    VisualUpdateType type;
};

const char *resultToStr(Result res);

Result search(
    const terrainmap::TerrainMap &map,
    Config &config,
    std::vector<terrainmap::Point2D> &path
);

bool createPath(
    const terrainmap::TerrainMap &map,
    uint32_t startPos,
    uint32_t destPos,
    const std::vector<uint32_t> &parents,
    std::vector<terrainmap::Point2D> &path
);

} // namespace pathfinder
