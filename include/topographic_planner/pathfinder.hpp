#pragma once

#include <cstdint>
#include <functional>
#include <grid_map_core/grid_map_core.hpp>
#include <string>
#include <vector>

namespace pathfinder {

struct Weights {
    float wxyDist;
    float wgrad;
    float wcost;
    float wheuristic;
    float maxGrad;
};

struct Config {
    Weights weights;
    grid_map::Position start;
    grid_map::Position dest;
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
    std::vector<grid_map::Position> &path
);

} // namespace pathfinder
