#include "topographic_planner/pathfinder.hpp"

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <queue>
#include <vector>

#define PF_COST_INFINITY FLT_MAX

namespace pathfinder {

class Solver {
    struct Cell {
        grid_map::Index index;
        float path_len;

        bool operator<(const Cell &other) const {
            return path_len > other.path_len;
        }
    };

  public:
    Solver(const grid_map::GridMap &map, const Config &config)
        : map_(map), config_(config), size_(map.getSize()) {
        int total_cells = size_(0) * size_(1);
        parents_.assign(total_cells, -1);
        closed_list_.assign(total_cells, false);
        dists_.assign(total_cells, PF_COST_INFINITY);
    }

    Result solve(
        const grid_map::Position &start,
        const grid_map::Position &dest,
        Path &path
    ) {
        if (!map_.getIndex(start, start_idx_))
            return Result::StartOutsideMap;
        if (!map_.getIndex(dest, goal_idx_))
            return Result::GoalOutsideMap;

        if (!run_search()) {
            if (was_cancelled_) {
                return Result::Cancelled;
            }
            return Result::NoPath;
        }

        if (!reconstruct_path(path))
            return Result::ErrMem;

        return Result::FoundPath;
    }

  private:
    int to_1d(const grid_map::Index &idx) const {
        return idx(0) * size_(1) + idx(1);
    }

    grid_map::Index to_2d(int idx_1d) const {
        return grid_map::Index(idx_1d / size_(1), idx_1d % size_(1));
    }

    bool is_traversable(float cost) const { return cost != PF_COST_INFINITY; }

    /* Distance between adjacent (orthogonal or diagonal) cells */
    float calc_adjacent_dist(
        const grid_map::Index &from_idx,
        const grid_map::Index &to_idx
    ) const {
        bool is_diagonal =
            (from_idx(0) != to_idx(0) && from_idx(1) != to_idx(1));
        return (is_diagonal ? static_cast<float>(M_SQRT2) : 1.0f) *
               map_.getResolution();
    }

    /* Signed gradient between two adjacent cells */
    float calc_grad(
        const grid_map::Index &from_idx,
        const grid_map::Index &to_idx,
        float xy_dist
    ) const {
        float curr_elev = map_.at(config_.elev_layer, from_idx);
        float next_elev = map_.at(config_.elev_layer, to_idx);
        if (std::isnan(curr_elev) || std::isnan(next_elev))
            return PF_COST_INFINITY;

        return next_elev - curr_elev / xy_dist;
    }

    /* Octile distance between cells */
    float calc_octile_dist(
        const grid_map::Index &from_idx,
        const grid_map::Index &to_idx
    ) const {
        grid_map::Position curr_pos, next_pos;
        map_.getPosition(from_idx, curr_pos);
        map_.getPosition(to_idx, next_pos);
        float dy = std::abs(curr_pos.y() - next_pos.y());
        float dx = std::abs(curr_pos.x() - next_pos.x());
        float octile_dist =
            std::max(dy, dx) +
            (static_cast<float>(M_SQRT2) - 1.0f) * std::min(dy, dx);
        return octile_dist;
    }

    float calc_edge_cost(
        const grid_map::Index &from_idx,
        const grid_map::Index &to_idx
    ) const {
        // Checks is within map bounds
        if (!map_.isValid(to_idx, config_.trav_layer))
            return PF_COST_INFINITY;

        // Checks is traversable
        const float cost = map_.at(config_.trav_layer, to_idx);
        if (cost >= LETHAL_COST || std::isnan(cost))
            return PF_COST_INFINITY;

        const float xy_dist = calc_adjacent_dist(from_idx, to_idx);
        const float abs_grad = std::abs(calc_grad(from_idx, to_idx, xy_dist));
        if (abs_grad >= config_.weights.max_grad)
            return PF_COST_INFINITY;

        return config_.weights.grad * abs_grad +
               config_.weights.xy_dist * xy_dist + config_.weights.cost * cost;
    }

    void relax_edge(
        const grid_map::Index &curr_idx,
        const grid_map::Index &next_idx
    ) {
        const float edge_cost = calc_edge_cost(curr_idx, next_idx);
        if (!is_traversable(edge_cost))
            return;

        int curr_1d = to_1d(curr_idx);
        int next_1d = to_1d(next_idx);

        const float dist_to_cell = dists_[curr_1d] + edge_cost;

        if (dist_to_cell >= dists_[next_1d])
            return;

        dists_[next_1d] = dist_to_cell;
        parents_[next_1d] = curr_1d;

        const float dist_to_goal =
            calc_octile_dist(next_idx, goal_idx_) * config_.weights.heuristic;
        open_list_.push(Cell{next_idx, dist_to_cell + dist_to_goal});
    }

    /* Updates the closed list with this node */
    void close_node(const grid_map::Index &idx) {
        int pos_1d = to_1d(idx);
        closed_list_[pos_1d] = true;
        if (config_.visualization_callback) {
            grid_map::Position p1, p2;
            int parent_1d = parents_[pos_1d];
            map_.getPosition(to_2d(parent_1d), p1);
            map_.getPosition(idx, p2);
            config_.visualization_callback(p1, p2, 0);
        }
    }

    bool run_search() {
        int start_1d = to_1d(start_idx_);
        parents_[start_1d] = start_1d;
        dists_[start_1d] = 0.0f;
        open_list_.push(Cell{start_idx_, 0.0f});

        int iter_cnt = 0;
        while (!open_list_.empty()) {
            // Periodically checks if the planner has been cancelled.
            if (++iter_cnt % 100 == 0 && config_.abort_check_callback &&
                config_.abort_check_callback()) {
                was_cancelled_ = true;
                return false;
            }

            Cell closest_cell = open_list_.top();
            open_list_.pop();

            grid_map::Index curr_idx = closest_cell.index;
            int curr_1d = to_1d(curr_idx);

            if (closed_list_[curr_1d])
                continue;

            if (!is_traversable(closest_cell.path_len))
                return false;

            close_node(curr_idx);

            // Reached goal
            if (curr_idx(0) == goal_idx_(0) && curr_idx(1) == goal_idx_(1))
                return true;

            // Relaxes the 8 adjacent cells
            for (int i = -1; i <= 1; ++i) {
                for (int j = -1; j <= 1; ++j) {
                    if (i == 0 && j == 0)
                        continue;       // Current cell

                    grid_map::Index next_idx(curr_idx(0) + i, curr_idx(1) + j);

                    if (!map_.isValid(next_idx))
                        continue;

                    if (closed_list_[to_1d(next_idx)])
                        continue;

                    relax_edge(curr_idx, next_idx);
                }
            }
        }
        return false;
    }

    bool reconstruct_path(Path &path) {
        path.clear();
        grid_map::Index curr_idx = goal_idx_;

        while (true) {
            grid_map::Position point;
            map_.getPosition(curr_idx, point);
            path.push_back(point);

            if (curr_idx(0) == start_idx_(0) && curr_idx(1) == start_idx_(1))
                break;

            int curr_1d = to_1d(curr_idx);
            int parent_1d = parents_[curr_1d];

            if (parent_1d == -1) {
                return false;    // Error finding parent
            }
            curr_idx = to_2d(parent_1d);
        }
        std::reverse(path.begin(), path.end());
        return true;
    }

    const grid_map::GridMap &map_;
    const Config &config_;
    grid_map::Size size_;
    grid_map::Index start_idx_;
    grid_map::Index goal_idx_;

    constexpr static float LETHAL_COST = 253.0f;
    bool was_cancelled_ = false;

    std::priority_queue<Cell> open_list_;
    std::vector<bool> closed_list_;
    std::vector<int> parents_;
    std::vector<float> dists_;
};

Result search(
    const grid_map::GridMap &map,
    const Config &config,
    const grid_map::Position &start,
    const grid_map::Position &dest,
    Path &path
) {
    Solver solver(map, config);
    return solver.solve(start, dest, path);
}

std::string_view result_description(Result res) {
    switch (res) {
    case Result::FoundPath:
        return "found a path from start to destination";
    case Result::NoPath:
        return "could not find a path from start to destination";
    case Result::ErrMem:
        return "failed to find a path due to an internal error";
    case Result::StartOutsideMap:
        return "start point is outside the map";
    case Result::GoalOutsideMap:
        return "goal point is outside the map";
    case Result::Cancelled:
        return "search was cancelled";
    default:
        return "unknown error";
    }
}

}    // namespace pathfinder
