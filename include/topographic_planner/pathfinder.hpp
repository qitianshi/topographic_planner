#pragma once

#include <grid_map_core/grid_map_core.hpp>

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace pathfinder {

struct Weights {
    float xy_dist;
    float grad;
    float cost;
    float heuristic;
    float max_grad;
};

struct Config {
    Weights weights;
    std::string elev_layer;
    std::string trav_layer;
    std::function<
        void(const grid_map::Position &, const grid_map::Position &, int)>
        visualization_callback;
    std::function<bool()> abort_check_callback;
};

using Path = std::vector<grid_map::Position>;

// Pathfinding result, mirrors ComputePathToPose::Result
enum class Result {
    FoundPath,
    NoPath,
    ErrMem,
    StartOutsideMap,
    GoalOutsideMap,
    Cancelled
};

std::string_view result_description(Result res);

Result search(
    const grid_map::GridMap &map,
    const Config &config,
    const grid_map::Position &start,
    const grid_map::Position &dest,
    Path &path
);

}    // namespace pathfinder
