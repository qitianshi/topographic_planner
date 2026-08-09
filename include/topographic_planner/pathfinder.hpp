#pragma once

#include <grid_map_core/grid_map_core.hpp>

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace pathfinder {

/** Weights and thresholds used by the A* edge cost function */
struct Weights {
    float xy_dist;      ///< Weight of horizontal 2D distance
    float grad;         ///< Weight of terrain slope gradient
    float trav;         ///< Weight of the traversability layer value
    float heuristic;    ///< Weight of the octile distance heuristic
    float max_grad;     ///< Maximum allowed slope gradient threshold
};

/** Configuration parameters for Pathfinder */
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

/**
 * @brief Represents the outcome of a pathfinding request.
 *
 * Subset of `nav2_msgs::action::ComputePathToPose`.
 */
enum class Result {
    FoundPath,
    NoPath,
    StartOutsideMap,
    StartOccupied,
    GoalOutsideMap,
    GoalOccupied,
    Cancelled
};

/** 2.5D A* path planner for grid_maps */
class Pathfinder {
  public:
    Pathfinder();
    ~Pathfinder();

    /**
     * @brief Executes an A* search on the given grid_map from start to dest.
     *
     * @param map The grid map containing elevation and traversability layers.
     * @param config Configuration object.
     * @param start The starting world coordinates.
     * @param dest The destination world coordinates.
     * @param path The output path, populated if a path is found.
     * @return Result enum indicating success or failure reason.
     */
    Result search(
        const grid_map::GridMap &map,
        const Config &config,
        const grid_map::Position &start,
        const grid_map::Position &dest,
        Path &path
    );

  private:
    // Pimpl idiom. Hides internal A* node and state allocation details.
    class Solver;
    std::unique_ptr<Solver> solver_;
};

}    // namespace pathfinder
