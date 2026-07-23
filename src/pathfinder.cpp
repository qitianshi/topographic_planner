#include "topographic_planner/pathfinder.hpp"
#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>

#define PF_SQRT_2 1.4142135623730950488016887242097f
#define PF_COST_INFINITY FLT_MAX

namespace pathfinder {

// Custom hash for grid_map::Index
struct IndexHash {
    std::size_t operator()(const grid_map::Index &idx) const {
        return std::hash<int>()(idx(0)) ^ (std::hash<int>()(idx(1)) << 1);
    }
};
struct IndexEqual {
    bool
    operator()(const grid_map::Index &lhs, const grid_map::Index &rhs) const {
        return lhs(0) == rhs(0) && lhs(1) == rhs(1);
    }
};

struct Cell {
    grid_map::Index index;
    float pathLen; // heuristic + dist to cell from start

    bool operator<(const Cell &other) const {
        return pathLen > other.pathLen; // min-heap based on pathLen
    }
};

struct Cells {
    std::priority_queue<Cell> toExplore;
    std::unordered_map<grid_map::Index, grid_map::Index, IndexHash, IndexEqual>
        parents;
    std::unordered_map<grid_map::Index, bool, IndexHash, IndexEqual> intree;
    std::unordered_map<grid_map::Index, bool, IndexHash, IndexEqual> inqueue;
    std::unordered_map<grid_map::Index, float, IndexHash, IndexEqual> dists;
    grid_map::Index startIndex;
    grid_map::Index destIndex;
};

static bool isReachable(float cost) { return cost != PF_COST_INFINITY; }

static float cellCalcXyDist(
    const grid_map::GridMap &map,
    const grid_map::Index &curIndex,
    const grid_map::Index &nextIndex
) {
    bool isDiagonal =
        (curIndex(0) != nextIndex(0) && curIndex(1) != nextIndex(1));
    return (isDiagonal ? PF_SQRT_2 : 1.0f) * map.getResolution();
}

static float cellCalcGrad(
    const grid_map::GridMap &map,
    const std::string &elevation_layer,
    const grid_map::Index &curIndex,
    const grid_map::Index &nextIndex,
    float xyDist
) {
    if (std::isnan(map.at(elevation_layer, curIndex)) ||
        std::isnan(map.at(elevation_layer, nextIndex))) {
        return PF_COST_INFINITY; // Unknown elevation means not traversable
    }
    const float elevDiff = std::abs(
        map.at(elevation_layer, curIndex) - map.at(elevation_layer, nextIndex)
    );
    return elevDiff / xyDist;
}

static float cellCalcDist(
    const grid_map::GridMap &map,
    const Weights &weights,
    const grid_map::Index &curIndex,
    const grid_map::Index &nextIndex
) {
    grid_map::Position curPoint, nextPoint;
    map.getPosition(curIndex, curPoint);
    map.getPosition(nextIndex, nextPoint);
    float dy = std::abs(curPoint.y() - nextPoint.y());
    float dx = std::abs(curPoint.x() - nextPoint.x());
    float octileDist = std::max(dy, dx) + (PF_SQRT_2 - 1.0f) * std::min(dy, dx);
    return octileDist * weights.wheuristic;
}

static float cellCalcEdgeCost(
    const grid_map::GridMap &map,
    const Config &config,
    const grid_map::Index &curIndex,
    const grid_map::Index &nextIndex
) {
    if (!map.isValid(nextIndex, config.traversability_layer))
        return PF_COST_INFINITY;
    const float cost = map.at(config.traversability_layer, nextIndex);
    if (cost >= 253.0f || std::isnan(cost)) // High cost threshold
        return PF_COST_INFINITY;

    const float xyDist = cellCalcXyDist(map, curIndex, nextIndex);
    const float grad =
        cellCalcGrad(map, config.elevation_layer, curIndex, nextIndex, xyDist);
    if (grad >= config.weights.maxGrad)
        return PF_COST_INFINITY;

    return config.weights.wgrad * grad + config.weights.wxyDist * xyDist +
           config.weights.wcost * cost;
}

static bool
cellsCreate(const grid_map::GridMap &map, const Config &config, Cells &cells) {
    if (!map.getIndex(config.start, cells.startIndex))
        return false;
    if (!map.getIndex(config.dest, cells.destIndex))
        return false;
    return true;
}

static void cellsEnqueue(
    const grid_map::GridMap &map,
    const Config &config,
    Cells &cells,
    const grid_map::Index &curIndex,
    const grid_map::Index &nextIndex
) {
    const float edgeCost = cellCalcEdgeCost(map, config, curIndex, nextIndex);
    if (!isReachable(edgeCost))
        return;

    auto it = cells.dists.find(curIndex);
    float curDist = (it != cells.dists.end()) ? it->second : PF_COST_INFINITY;
    const float distToCell = curDist + edgeCost;

    it = cells.dists.find(nextIndex);
    float nextDist = (it != cells.dists.end()) ? it->second : PF_COST_INFINITY;
    if (distToCell >= nextDist)
        return; // Not better

    cells.inqueue[nextIndex] = true;
    cells.dists[nextIndex] = distToCell;
    cells.parents[nextIndex] = curIndex;

    const float distToDest =
        cellCalcDist(map, config.weights, nextIndex, cells.destIndex);
    cells.toExplore.push(Cell{nextIndex, distToCell + distToDest});
}

static void cellsTreeAdd(
    const Config &config,
    const grid_map::GridMap &map,
    Cells &cells,
    const grid_map::Index &pos
) {
    cells.intree[pos] = true;
    if (config.visualization_callback) {
        grid_map::Position p1, p2;
        map.getPosition(cells.parents[pos], p1);
        map.getPosition(pos, p2);
        config.visualization_callback(p1, p2, 0); // 0 = Tree
    }
}

static bool searchInternal(
    const grid_map::GridMap &map, const Config &config, Cells &cells
) {
    grid_map::Index curIndex = cells.startIndex;
    cells.parents[curIndex] = curIndex;
    cells.dists[curIndex] = 0.0f;
    cells.toExplore.push(Cell{curIndex, 0.0f});
    cells.inqueue[curIndex] = true;

    while (!cells.toExplore.empty()) {
        Cell closestCell = cells.toExplore.top();
        cells.toExplore.pop();

        curIndex = closestCell.index;

        if (cells.intree[curIndex])
            continue;

        if (!isReachable(closestCell.pathLen)) {
            return false;
        }

        cellsTreeAdd(config, map, cells, curIndex);

        if (curIndex(0) == cells.destIndex(0) &&
            curIndex(1) == cells.destIndex(1)) {
            return true;
        }

        // 8-connected neighborhood
        for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
                if (i == 0 && j == 0)
                    continue;
                grid_map::Index nextIndex(curIndex(0) + i, curIndex(1) + j);

                // check if inside map
                if (!map.isValid(nextIndex))
                    continue;

                if (cells.intree[nextIndex])
                    continue;

                cellsEnqueue(map, config, cells, curIndex, nextIndex);
            }
        }
    }
    return false;
}

bool createPath(
    const grid_map::GridMap &map,
    const grid_map::Index &startPos,
    const grid_map::Index &destPos,
    const std::
        unordered_map<grid_map::Index, grid_map::Index, IndexHash, IndexEqual>
            &parents,
    std::vector<grid_map::Position> &path
) {
    path.clear();
    grid_map::Index curPos = destPos;
    while (true) {
        grid_map::Position point;
        map.getPosition(curPos, point);
        path.push_back(point);
        if (curPos(0) == startPos(0) && curPos(1) == startPos(1))
            break;
        auto it = parents.find(curPos);
        if (it == parents.end()) {
            return false; // Error finding parent
        }
        curPos = it->second;
    }
    std::reverse(path.begin(), path.end());
    return true;
}

Result search(
    const grid_map::GridMap &map,
    const Config &config,
    std::vector<grid_map::Position> &path
) {
    Cells cells;
    if (!cellsCreate(map, config, cells)) {
        return Result::StartOutsideMap; // Assuming issue is start/dest pos
    }

    if (!searchInternal(map, config, cells)) {
        return Result::NoPath;
    }

    if (!createPath(
            map, cells.startIndex, cells.destIndex, cells.parents, path
        )) {
        return Result::ErrMem;
    }

    return Result::FoundPath;
}

const char *resultToStr(Result res) {
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
    default:
        return "unknown error";
    }
}

} // namespace pathfinder
