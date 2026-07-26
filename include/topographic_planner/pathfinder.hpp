#pragma once

#include <grid_map_core/grid_map_core.hpp>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace pathfinder {

struct Weights {
    float xyDist;
    float grad;
    float cost;
    float heuristic;
    float maxGrad;
};

struct Config {
    Weights weights;
    std::string elevation_layer;
    std::string traversability_layer;
    std::function<
        void(const grid_map::Position &, const grid_map::Position &, int)>
        visualization_callback;
};

enum class Result {
    FoundPath,
    NoPath,
    ErrMem,
    StartOutsideMap,
    GoalOutsideMap
};

const char *resultToStr(Result res);

Result search(
    const grid_map::GridMap &map,
    const Config &config,
    const grid_map::Position &start,
    const grid_map::Position &dest,
    std::vector<grid_map::Position> &path
);

} // namespace pathfinder
