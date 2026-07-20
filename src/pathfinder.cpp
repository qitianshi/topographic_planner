#include "pathfinder.hpp"
#include <cassert>
#include <cfloat>
#include <cmath>
#include <queue>
#include <vector>
#include <algorithm>

#define PF_SQRT_2 1.4142135623730950488016887242097f
#define PF_COST_INFINITY FLT_MAX
#define PF_NEIGHBOURHOOD_SIZE 3

namespace pathfinder {

struct Cell {
    uint32_t pos;
    terrainmap::PhysDist pathLen; // heuristic + dist to cell from start

    bool operator<(const Cell& other) const {
        return pathLen > other.pathLen; // min-heap based on pathLen
    }
};

struct Cells {
    std::priority_queue<Cell> toExplore;
    std::vector<uint32_t> parents;
    std::vector<bool> intree;
    std::vector<bool> inqueue; // to track if a node is ever placed in queue
    std::vector<float> dists;
    uint32_t startPos;
    uint32_t destPos;
};

static void visualUpdateChangeEdge(
    const ExtraOut& out,
    uint32_t curPos,
    uint32_t nextPos,
    uint32_t oldParent
) {
    if (!out.visual) return;
    VisualUpdate update = {
        nextPos,
        curPos,
        oldParent,
        VisualUpdateType::ChangeEdge,
    };
    fwrite(&update, sizeof(update), 1, out.visual);
}

static void visualUpdateQueue(
    const ExtraOut& out,
    uint32_t curPos,
    uint32_t nextPos
) {
    if (!out.visual) return;
    VisualUpdate update = {
        nextPos,
        curPos,
        0, // padding or unused for queue
        VisualUpdateType::Queue
    };
    fwrite(&update, sizeof(update), 1, out.visual);
}

static void visualUpdateTree(
    const ExtraOut& out,
    uint32_t curPos,
    uint32_t nextPos
) {
    if (!out.visual) return;
    VisualUpdate update = {
        nextPos,
        curPos,
        0,
        VisualUpdateType::Tree
    };
    fwrite(&update, sizeof(update), 1, out.visual);
}

static void visualUpdateFinished(
    const ExtraOut& out
) {
    if (!out.visual) return;
    VisualUpdate update = {};
    update.type = VisualUpdateType::Finished;
    fwrite(&update, sizeof(update), 1, out.visual);
}

static bool isReachable(float cost) {
    return cost != PF_COST_INFINITY;
}

static terrainmap::PhysDist cellCalcXyDist(
    const terrainmap::TerrainMap& map,
    uint32_t curPos,
    uint32_t nextPos
) {
    return (map.isDiagonal(curPos, nextPos) ? PF_SQRT_2 : 1.0f) * map.resolution;
}

static terrainmap::PhysDist cellCalcGrad(
    const terrainmap::TerrainMap& map,
    uint32_t curPos,
    uint32_t nextPos,
    terrainmap::PhysDist xyDist
) {
    const float elevDiff = std::abs(map.dataElevation[curPos] - map.dataElevation[nextPos]);
    return elevDiff / xyDist;
}

static terrainmap::PhysDist cellCalcDist(
    const terrainmap::TerrainMap& map,
    const Weights& weights,
    uint32_t curPos,
    uint32_t nextPos
) {
    terrainmap::GridCoord curCoord = map.getCoord(curPos);
    terrainmap::GridCoord nextCoord = map.getCoord(nextPos);
    terrainmap::Point2D curPoint = map.toPoint(curCoord);
    terrainmap::Point2D nextPoint = map.toPoint(nextCoord);
    terrainmap::PhysDist dy = std::abs(curPoint.y - nextPoint.y);
    terrainmap::PhysDist dx = std::abs(curPoint.x - nextPoint.x);
    terrainmap::PhysDist octileDist = std::max(dy, dx) + (PF_SQRT_2 - 1.0f) * std::min(dy, dx);
    return octileDist * weights.wheuristic;
}

static float cellCalcEdgeCost(
    const terrainmap::TerrainMap& map,
    const Weights& weights,
    const uint32_t curPos,
    const uint32_t nextPos
) {
    const uint8_t cost = map.dataCost[nextPos];
    if (cost == UINT8_MAX) return PF_COST_INFINITY;

    const float xyDist = cellCalcXyDist(map, curPos, nextPos);
    const float grad = cellCalcGrad(map, curPos, nextPos, xyDist);
    if (grad >= weights.maxGrad) return PF_COST_INFINITY;

    return weights.wgrad * grad + weights.wxyDist * xyDist + weights.wcost * cost;
}

static bool cellsCreate(
    const terrainmap::TerrainMap& map,
    const Config& config,
    Cells& cells
) {
    const size_t npoints = map.getNpoints();
    try {
        cells.parents.resize(npoints);
        cells.intree.resize(npoints, false);
        cells.inqueue.resize(npoints, false);
        cells.dists.resize(npoints, PF_COST_INFINITY);
    } catch (...) {
        return false;
    }

    cells.startPos = map.getPos(config.start);
    cells.destPos = map.getPos(config.dest);
    return true;
}

static void cellsEnqueue(
    const terrainmap::TerrainMap& map,
    const Config& config,
    Cells& cells,
    uint32_t curPos,
    uint32_t nextPos
) {
    const float edgeCost = cellCalcEdgeCost(map, config.weights, curPos, nextPos);
    if (!isReachable(edgeCost)) return;

    const terrainmap::PhysDist distToCell = cells.dists[curPos] + edgeCost;
    if (distToCell >= cells.dists[nextPos]) return; // Not better

    if (cells.inqueue[nextPos]) {
        visualUpdateChangeEdge(config.extraOut, curPos, nextPos, cells.parents[nextPos]);
    } else {
        visualUpdateQueue(config.extraOut, curPos, nextPos);
        cells.inqueue[nextPos] = true;
    }

    cells.dists[nextPos] = distToCell;
    cells.parents[nextPos] = curPos;

    const float distToDest = cellCalcDist(map, config.weights, nextPos, cells.destPos);
    cells.toExplore.push(Cell{nextPos, distToCell + distToDest});
}

static void displaySearch(
    const terrainmap::TerrainMap& map,
    const Config* config,
    const Cells& cells,
    const std::vector<terrainmap::Point2D>* path
) {
    FILE* out = nullptr;
    if (config && config->extraOut.searchDisplay) {
        out = config->extraOut.searchDisplay;
    } else if (!config) {
        out = stdout;
    }
    if (!out) return;

    size_t npoints = map.getNpoints();
    std::vector<char> buf(npoints + map.npointsY + 1);
    char* head = buf.data();

    for (size_t y = 0; y < map.npointsY; ++y) {
        for (size_t x = 0; x < map.npointsX; ++x) {
            size_t pos = y * map.npointsX + x;
            char c = '-';
            if (cells.intree[pos]) {
                c = '+';
            } else if (cells.inqueue[pos]) {
                c = '#';
            }
            *head++ = c;
        }
        *head++ = '\n';
    }
    *head++ = '\n';

    if (path) {
        for (const auto& point : *path) {
            uint32_t pos = map.getPos(point);
            pos += pos / map.npointsX; // Adjust for newlines
            buf[pos] = 'O';
        }
    }

    fwrite(buf.data(), 1, static_cast<size_t>(head - buf.data()), out);
    if (out != stdout) fflush(out);
}

static void cellsTreeAdd(
    const Config& config,
    Cells& cells,
    uint32_t pos
) {
    cells.intree[pos] = true;
    visualUpdateTree(config.extraOut, cells.parents[pos], pos);
}

static bool searchInternal(
    const terrainmap::TerrainMap& map,
    const Config& config,
    Cells& cells
) {
    uint32_t curPos = cells.startPos;
    cells.parents[curPos] = curPos;
    cells.dists[curPos] = 0.0f;
    cells.toExplore.push(Cell{curPos, 0.0f});
    cells.inqueue[curPos] = true;

    while (!cells.toExplore.empty()) {
        Cell closestCell = cells.toExplore.top();
        cells.toExplore.pop();

        curPos = closestCell.pos;

        if (cells.intree[curPos]) continue;

        if (!isReachable(closestCell.pathLen)) {
            return false;
        }

        cellsTreeAdd(config, cells, curPos);

        if (curPos == cells.destPos) {
            return true;
        }

        for (uint32_t rowPos = curPos - 1 - map.npointsX,
                      lastRowPos = curPos + map.npointsX - 1;
             rowPos <= lastRowPos; rowPos += map.npointsX) {
            for (uint32_t nextPos = rowPos, stopPos = rowPos + PF_NEIGHBOURHOOD_SIZE;
                 nextPos < stopPos; ++nextPos) {
                if (!map.contains(nextPos)) continue;
                if (nextPos == curPos) continue;
                if (cells.intree[nextPos]) continue;

                cellsEnqueue(map, config, cells, curPos, nextPos);
            }
        }
    }
    return false;
}

bool createPath(
    const terrainmap::TerrainMap& map,
    uint32_t startPos,
    uint32_t destPos,
    const std::vector<uint32_t>& parents,
    std::vector<terrainmap::Point2D>& path
) {
    path.clear();
    for (uint32_t curPos = destPos; curPos != UINT32_MAX; curPos = parents[curPos]) {
        terrainmap::GridCoord coord = map.getCoord(curPos);
        terrainmap::Point2D point = map.toPoint(coord);
        path.push_back(point);
        if (curPos == startPos) break;
    }
    std::reverse(path.begin(), path.end());
    return true;
}

static void evalPath(
    const terrainmap::TerrainMap& map,
    const Config& config,
    const Cells& cells,
    const std::vector<terrainmap::Point2D>& path
) {
    if (path.empty()) return;

    FILE* out = config.extraOut.pathEval ? config.extraOut.pathEval : stdout;
    if (!out) return;

    uint32_t prevPos = cells.startPos;
    terrainmap::PhysDist totalDistFromDest = 0;
    terrainmap::PhysDist totalDistFromStart = 0;
    terrainmap::PhysDist txy = 0, tg = 0, tc = 0;
    terrainmap::PhysDist twxy = 0, twg = 0, twc = 0;

    terrainmap::Point2D minEdgePoint = {NAN, NAN};
    terrainmap::Point2D maxEdgePoint = {NAN, NAN};
    float minEdgeCost = FLT_MAX;
    float maxEdgeCost = FLT_MIN;
    const Weights& weights = config.weights;

    for (const auto& point : path) {
        uint32_t pos = map.getPos(point);
        terrainmap::GridCoord coord = map.getCoord(pos);
        float edgeCost = cellCalcEdgeCost(map, weights, prevPos, pos);
        terrainmap::PhysDist distToDest = cellCalcDist(map, weights, pos, cells.destPos);
        terrainmap::PhysDist xyDist = cellCalcXyDist(map, prevPos, pos);
        terrainmap::PhysDist grad = cellCalcGrad(map, prevPos, pos, xyDist);
        terrainmap::PhysDist cost = map.dataCost[pos];

        terrainmap::PhysDist wxyDist = weights.wxyDist * xyDist;
        terrainmap::PhysDist wgrad = weights.wgrad * grad;
        terrainmap::PhysDist wcost = weights.wcost * cost;
        float dist = cells.dists[pos];

        totalDistFromDest += distToDest;
        totalDistFromStart += dist;
        txy += xyDist;
        tg += grad;
        tc += cost;
        twxy += wxyDist;
        twg += wgrad;
        twc += wcost;

        if (edgeCost < minEdgeCost) {
            minEdgePoint = point;
            minEdgeCost = edgeCost;
        }
        if (edgeCost > maxEdgeCost) {
            maxEdgePoint = point;
            maxEdgeCost = edgeCost;
        }
        prevPos = pos;

        fprintf(out,
            "%6d (%6d, %6d) (%4.1f, %4.1f) (xy=%6.2f, g=%6.2f, c=%7.2f) "
            "(wxy=%6.2f, wg=%6.2f. wc=%6.2f) (ec=%6.2f, d=%6.2f, h=%6.2f)\n",
            (int)pos, (int)coord.x, (int)coord.y, point.x, point.y, xyDist,
            grad, cost, wxyDist, wgrad, wcost, edgeCost, dist, distToDest
        );
    }

    float pathLen = static_cast<float>(path.size());
    totalDistFromStart /= pathLen;
    totalDistFromDest /= pathLen;
    txy /= pathLen;
    tg /= pathLen;
    tc /= pathLen;
    twxy /= pathLen;
    twg /= pathLen;
    twc /= pathLen;

    fprintf(out, "min edge cost  : %7.2f (%4.1f, %4.1f)\n", minEdgeCost, minEdgePoint.x, minEdgePoint.y);
    fprintf(out, "max edge cost  : %7.2f (%4.1f, %4.1f)\n", maxEdgeCost, maxEdgePoint.x, maxEdgePoint.y);
    fprintf(out, "avg dist strt  : %7.2f\n", totalDistFromStart);
    fprintf(out, "avg dist dest  : %7.2f\n", totalDistFromDest);
    fprintf(out, "ratio src/dest : %7.2f\n", totalDistFromStart / totalDistFromDest);
    fprintf(out, "avg (xy, g, c) : (xy= %6.2f, g= %6.2f, c= %6.2f)\n", txy, tg, tc);
    fprintf(out, "avg (wxy,wg,wc): (wxy=%6.2f, wg=%6.2f. wc=%6.2f)\n", twxy, twg, twc);

    if (out != stdout) fflush(out);
}

Result search(
    const terrainmap::TerrainMap& map,
    Config& config,
    std::vector<terrainmap::Point2D>& path
) {
    Cells cells;
    if (!cellsCreate(map, config, cells)) {
        return Result::ErrMem;
    }

    if (!searchInternal(map, config, cells)) {
        visualUpdateFinished(config.extraOut);
        return Result::NoPath;
    }

    if (!createPath(map, cells.startPos, cells.destPos, cells.parents, path)) {
        visualUpdateFinished(config.extraOut);
        return Result::ErrMem;
    }

    displaySearch(map, &config, cells, &path);
    evalPath(map, config, cells, path);
    visualUpdateFinished(config.extraOut);
    return Result::FoundPath;
}

const char* resultToStr(Result res) {
    switch(res) {
        case Result::FoundPath: return "found a path from start to destination";
        case Result::NoPath: return "could not find a path from start to destination";
        case Result::ErrMem: return "failed to find a path due to a memory allocation error";
        default: return "unknown error";
    }
}

}
