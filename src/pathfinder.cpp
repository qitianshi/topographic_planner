#include "topographic_planner/pathfinder.hpp"

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <vector>

namespace pathfinder {

/** A* search solver */
class Pathfinder::Solver {

    /** A node in the A* search tree, containing the grid index and f-score */
    struct Node {
        grid_map::Index index;
        float f_score;

        bool operator<(const Node &other) const {
            return f_score > other.f_score;
        }
    };

  public:
    Solver() = default;

    Result solve(
        const grid_map::GridMap &map,
        const Config &config,
        const grid_map::Position &start,
        const grid_map::Position &dest,
        Path &path
    ) {
        map_ = &map;
        config_ = &config;
        map_size_ = map_->getSize();

        // Reallocates only if the map size changes; otherwise, resets the
        // existing allocation.
        int total_nodes = map_size_(0) * map_size_(1);
        if (parents_.size() != static_cast<size_t>(total_nodes)) {
            parents_.assign(total_nodes, parents_null_val_);
            closed_list_.assign(total_nodes, false);
            g_scores_.assign(total_nodes, edge_cost_infinity);
        } else {
            std::fill(parents_.begin(), parents_.end(), parents_null_val_);
            std::fill(closed_list_.begin(), closed_list_.end(), false);
            std::fill(g_scores_.begin(), g_scores_.end(), edge_cost_infinity);
        }

        std::priority_queue<Node> empty;
        std::swap(open_list_, empty);
        was_cancelled_ = false;

        // Precomputes and caches distance for performance
        map_res_ortho_ = map_->getResolution();
        map_res_diag_ = static_cast<float>(M_SQRT2) * map_res_ortho_;

        // Obtains direct references to elev and trav layers for performance
        elev_data_ = &map_->get(config_->elev_layer);
        trav_data_ = &map_->get(config_->trav_layer);

        // Checks start and goal are within map bounds
        if (!map_->getIndex(start, start_idx_))
            return Result::StartOutsideMap;
        if (!map_->getIndex(dest, goal_idx_))
            return Result::GoalOutsideMap;

        // Checks start and goal are traversable
        const float start_trav = (*trav_data_)(start_idx_(0), start_idx_(1));
        const float goal_trav = (*trav_data_)(goal_idx_(0), goal_idx_(1));
        if (std::isnan(start_trav) || start_trav >= lethal_trav)
            return Result::StartOccupied;
        if (std::isnan(goal_trav) || goal_trav >= lethal_trav)
            return Result::GoalOccupied;

        if (!run_search()) {
            if (was_cancelled_) {
                return Result::Cancelled;
            }
            return Result::NoPath;
        }

        reconstruct_path(path);

        return Result::FoundPath;
    }

  private:
    /** Converts a 2D `grid_map::Index` into a 1D flat array integer index */
    int to_1d(const grid_map::Index &idx) const {
        return idx(0) * map_size_(1) + idx(1);
    }

    /** Converts a 1D flat array integer index into a 2D `grid_map::Index` */
    grid_map::Index to_2d(int idx_1d) const {
        return grid_map::Index(idx_1d / map_size_(1), idx_1d % map_size_(1));
    }

    /** Calculates the 2D Euclidean distance between adjacent nodes */
    float adjacent_dist(
        const grid_map::Index &from_idx,
        const grid_map::Index &to_idx
    ) const {
        bool is_diagonal =
            (from_idx(0) != to_idx(0) && from_idx(1) != to_idx(1));
        return is_diagonal ? map_res_diag_ : map_res_ortho_;
    }

    /** Calculates the signed elevation gradient between two adjacent nodes */
    float adjacent_grad(
        const grid_map::Index &from_idx,
        const grid_map::Index &to_idx,
        float xy_dist
    ) const {
        float curr_elev = (*elev_data_)(from_idx(0), from_idx(1));
        float next_elev = (*elev_data_)(to_idx(0), to_idx(1));
        if (std::isnan(curr_elev) || std::isnan(next_elev))
            return edge_cost_infinity;

        return (next_elev - curr_elev) / xy_dist;
    }

    /** Calculates the traversal cost for a single edge */
    float calc_edge_cost(
        const grid_map::Index &from_idx,
        const grid_map::Index &to_idx
    ) const {
        // Checks is within map bounds
        if (!map_->isValid(to_idx, config_->trav_layer))
            return edge_cost_infinity;

        const float trav = (*trav_data_)(to_idx(0), to_idx(1));
        if (std::isnan(trav) || trav >= lethal_trav)
            return edge_cost_infinity;

        const float adj_dist = adjacent_dist(from_idx, to_idx);
        const float abs_grad =
            std::abs(adjacent_grad(from_idx, to_idx, adj_dist));
        if (abs_grad >= config_->weights.max_grad)
            return edge_cost_infinity;

        return config_->weights.grad * abs_grad +
               config_->weights.xy_dist * adj_dist +
               config_->weights.trav * trav;
    }

    /** Calculates the A* heuristic (octile distance) between two indices */
    float calc_heuristic(
        const grid_map::Index &from_idx,
        const grid_map::Index &to_idx
    ) const {
        // Index calculation avoids computing map coordinates
        float dy = std::abs(from_idx(1) - to_idx(1)) * map_res_ortho_;
        float dx = std::abs(from_idx(0) - to_idx(0)) * map_res_ortho_;
        float octile_dist =
            std::max(dy, dx) +
            (static_cast<float>(M_SQRT2) - 1.0f) * std::min(dy, dx);
        return octile_dist;
    }

    /**
     * @brief Evaluates and potentially relaxes an edge during A* expansion.
     * @invariant `from_idx` and `to_idx` must be adjacent.
     */
    void relax_edge(
        const grid_map::Index &curr_idx,
        const grid_map::Index &next_idx
    ) {
        const float edge_cost = calc_edge_cost(curr_idx, next_idx);
        if (edge_cost == edge_cost_infinity)
            return;

        int curr_1d = to_1d(curr_idx);
        int next_1d = to_1d(next_idx);

        const float tentative_g = g_scores_[curr_1d] + edge_cost;

        if (tentative_g >= g_scores_[next_1d])
            return;

        // If moving to next_idx from curr_idx yields a lower g_score than
        // previously known: updates the g_score, sets the parent, and pushes it
        // to the open list.

        g_scores_[next_1d] = tentative_g;
        parents_[next_1d] = curr_1d;

        const float h_score =
            calc_heuristic(next_idx, goal_idx_) * config_->weights.heuristic;
        open_list_.push(Node{next_idx, tentative_g + h_score});
    }

    /** Marks a node as expanded (closed) and fires the debug callback */
    void close_node(const grid_map::Index &idx) {
        int pos_1d = to_1d(idx);
        closed_list_[pos_1d] = true;
        if (config_->visualization_callback) {
            grid_map::Position p1, p2;
            int parent_1d = parents_[pos_1d];
            map_->getPosition(to_2d(parent_1d), p1);
            map_->getPosition(idx, p2);
            config_->visualization_callback(p1, p2, 0);
        }
    }

    /** Runs the A* search algorithm */
    bool run_search() {
        int start_1d = to_1d(start_idx_);
        parents_[start_1d] = start_1d;
        g_scores_[start_1d] = 0.0f;
        open_list_.push(Node{start_idx_, 0.0f});

        int iter_cnt = 0;
        while (!open_list_.empty()) {
            // Periodically checks if the planner has been cancelled.
            if (++iter_cnt % 100 == 0 && config_->abort_check_callback &&
                config_->abort_check_callback()) {
                was_cancelled_ = true;
                return false;
            }

            Node frontier_node = open_list_.top();
            open_list_.pop();
            grid_map::Index frontier_idx = frontier_node.index;
            int curr_1d = to_1d(frontier_idx);

            if (closed_list_[curr_1d])
                continue;
            if (frontier_node.f_score == edge_cost_infinity)
                return false;

            close_node(frontier_idx);

            // Reached goal
            if (frontier_idx(0) == goal_idx_(0) &&
                frontier_idx(1) == goal_idx_(1))
                return true;

            // Relaxes the 8 adjacent nodes
            for (int i = -1; i <= 1; ++i) {
                for (int j = -1; j <= 1; ++j) {
                    if (i == 0 && j == 0)
                        continue;    // The frontier node itself

                    grid_map::Index next_idx(
                        frontier_idx(0) + i, frontier_idx(1) + j
                    );

                    if (!map_->isValid(next_idx))
                        continue;

                    if (closed_list_[to_1d(next_idx)])
                        continue;

                    relax_edge(frontier_idx, next_idx);
                }
            }
        }
        return false;
    }

    /** Reconstructs the final path */
    void reconstruct_path(Path &path) {
        path.clear();
        int curr_1d = to_1d(goal_idx_);
        int start_1d = to_1d(start_idx_);

        while (curr_1d != start_1d) {
            if (curr_1d == parents_null_val_) {
                throw std::logic_error(
                    "Path reconstruction failed: unexpected null parent"
                );
            }

            grid_map::Position point;
            map_->getPosition(to_2d(curr_1d), point);
            path.push_back(point);
            curr_1d = parents_[curr_1d];
        }

        // Add the start node itself
        grid_map::Position start_point;
        map_->getPosition(start_idx_, start_point);
        path.push_back(start_point);

        std::reverse(path.begin(), path.end());
    }

    const grid_map::GridMap *map_{nullptr};
    const Config *config_{nullptr};

    // Cached properties
    grid_map::Size map_size_;
    const grid_map::Matrix *trav_data_{nullptr};
    const grid_map::Matrix *elev_data_{nullptr};
    float map_res_ortho_{0.0f};
    float map_res_diag_{0.0f};

    // Search params
    grid_map::Index start_idx_;
    grid_map::Index goal_idx_;
    bool was_cancelled_ = false;

    // Constants
    constexpr static float lethal_trav = 253.0f;
    constexpr static int parents_null_val_ = -1;
    constexpr static float edge_cost_infinity = INFINITY;

    // Search state
    std::priority_queue<Node> open_list_;
    std::vector<bool> closed_list_;
    std::vector<int> parents_;
    std::vector<float> g_scores_;
};

Pathfinder::Pathfinder() : solver_(std::make_unique<Solver>()) {}

Pathfinder::~Pathfinder() = default;

Result Pathfinder::search(
    const grid_map::GridMap &map,
    const Config &config,
    const grid_map::Position &start,
    const grid_map::Position &dest,
    Path &path
) {
    return solver_->solve(map, config, start, dest, path);
}

}    // namespace pathfinder
