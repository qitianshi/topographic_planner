#include "adt/dynarr.h"
#include "adt/heap.h"
#include "adt/bitarr.h"
#include "map.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>

#define PF_EVAL_PATH 0
#define PF_DISPLAY_SEARCH 0

#define PF_SQRT_2 1.4142135623730950488016887242097f
#define PF_COST_INFINITY FLT_MAX

#define PF_WEIGHT_GRAD 1.26f
#define PF_WEIGHT_XY_DIST 1.34f
#define PF_WEIGHT_COST (1.81f * 1.0f / UINT8_MAX)
#define PF_WEIGHT_HEURISTIC 1.74f
#define PF_MAX_GRADIENT 0.577f    //tan(30) = 0.577
#define PF_NEIGHBOURHOOD_SIZE 3
#define PF_PATH_INITIAL_CAPACITY 128

#define PF_NOT_IN_HEAP 0

struct pf_cell_t {
    uint32_t pos;
    phys_dist_t path_len; //heuristic + dist to cell from start
};

struct pf_cells_t {
    struct heap_t to_explore;
    uint32_t *parents;
    uint32_t *heap_idxs;
    unsigned char* intree;
    float *dists;
};

static bool pf_is_reachable(float cost) {
    return cost != PF_COST_INFINITY;
}

static int pf_cell_cmp(const void* c1, const void* c2, void* state){
    const struct pf_cell_t* cell1 = c1;
    const struct pf_cell_t* cell2 = c2;
    return cell1->path_len < cell2->path_len ? 1 : -1;
}

static void pf_cells_queue_str_cell_idx(const void* vcell, size_t idx, void* state){
    const struct pf_cell_t *cell = vcell;
    uint32_t *heap_idx = state;
    heap_idx[cell->pos] = (uint32_t)idx + (PF_NOT_IN_HEAP + 1);
}

static bool pf_cells_tree_contains(struct pf_cells_t *cells, size_t pos) {
    return bitarr_get(cells->intree, pos);
}

static bool pf_cells_queue_contains(struct pf_cells_t *cells, size_t pos) {
    return cells->heap_idxs[pos] != PF_NOT_IN_HEAP;
}

static bool pf_cells_queue_get_idx(struct pf_cells_t* cells, size_t pos){
    return cells->heap_idxs[pos] - (PF_NOT_IN_HEAP + 1);
}

static phys_dist_t pf_cell_calc_xy_dist(
    const struct terrain_map_t *map,
    uint32_t cur_pos,
    uint32_t next_pos
){
    return (terrain_map_is_diagonal(map, cur_pos, next_pos) ? PF_SQRT_2 : 1) * map->resolution;
}

static phys_dist_t pf_cell_calc_grad(
    const struct terrain_map_t *map,
    uint32_t cur_pos,
    uint32_t next_pos,
    phys_dist_t xy_dist
){
    const float elev_diff = fabsf(map->data_elevation[cur_pos] - map->data_elevation[next_pos]);
    return elev_diff / xy_dist;
}

static phys_dist_t pf_cell_calc_dist(
    const struct terrain_map_t *map,
    uint32_t cur_pos,
    uint32_t next_pos
){
    struct grid_coord_t cur_coord = terrain_map_get_coord(map, cur_pos);
    struct grid_coord_t next_coord = terrain_map_get_coord(map, next_pos);
    struct point_2d_t cur_point = terrain_map_to_point(map, cur_coord);
    struct point_2d_t next_point = terrain_map_to_point(map, next_coord);
    phys_dist_t dy = fabsf(cur_point.y - next_point.y);
    phys_dist_t dx = fabsf(cur_point.x - next_point.x);
    phys_dist_t octile_dist = fmaxf(dy, dx) + (PF_SQRT_2 - 1) * fminf(dy, dx);
    return octile_dist * PF_WEIGHT_HEURISTIC;
}

static float pf_cell_calc_edge_cost(
    const struct terrain_map_t *map,
    const uint32_t cur_pos,
    const uint32_t next_pos
){
    const uint8_t cost = map->data_cost[next_pos];
    if (map->data_cost[next_pos] == UINT8_MAX)
        return PF_COST_INFINITY;
    const float xy_dist = pf_cell_calc_xy_dist(map, cur_pos, next_pos);
    const float grad = pf_cell_calc_grad(map, cur_pos, next_pos, xy_dist);
    if (grad >= PF_MAX_GRADIENT)
        return PF_COST_INFINITY;
    return PF_WEIGHT_GRAD * grad
        + PF_WEIGHT_XY_DIST * xy_dist
        + PF_WEIGHT_COST * cost;
}

static bool pf_cells_create(
    const struct terrain_map_t *map,
    struct pf_cells_t *cells
) {
    const size_t npoints = terrain_map_get_npoints(map);
    cells->parents = malloc(2 * npoints * sizeof *cells->parents);
    cells->heap_idxs = cells->parents + npoints;
    cells->intree = bitarr_make(npoints);
    cells->dists = malloc(npoints * sizeof *cells->dists);
    const bool success = heap_make(
        &cells->to_explore, npoints, sizeof(struct pf_cell_t), pf_cell_cmp,
        pf_cells_queue_str_cell_idx, cells->heap_idxs
    );
    return cells->parents && cells->heap_idxs && cells->intree &&
           cells->dists && success;
}

static void pf_cells_free(struct pf_cells_t *cells){
    free(cells->parents);
    bitarr_free(cells->intree);
    free(cells->dists);
    heap_free(&cells->to_explore);
}

static void pf_cells_enqueue(
    const struct terrain_map_t *map,
    struct pf_cells_t *cells,
    uint32_t cur_pos,
    uint32_t next_pos,
    uint32_t dest_pos
) {
    const float edge_cost = pf_cell_calc_edge_cost(map, cur_pos, next_pos);
    if (!pf_is_reachable(edge_cost))
        return;
    cells->dists[next_pos] = edge_cost + cells->dists[cur_pos];
    const float dist_to_dest = pf_cell_calc_dist(map, next_pos, dest_pos);
    struct pf_cell_t cell = {
        .pos = next_pos,
        .path_len = cells->dists[next_pos] + dist_to_dest
    };
    cells->parents[next_pos] = cur_pos;
    const bool success = heap_insert(&cells->to_explore, &cell);
    assert(success); // should never need to resize
}

static void pf_cells_consider_shorter_dist(
    const struct terrain_map_t *map,
    struct pf_cells_t *cells,
    uint32_t cur_pos,
    uint32_t next_pos,
    uint32_t dest_pos
){
    const float edge_cost = pf_cell_calc_edge_cost(map, cur_pos, next_pos);
    if (!pf_is_reachable(edge_cost))
        return;
    const phys_dist_t dist_to_cell = cells->dists[cur_pos] + edge_cost;
    if (dist_to_cell >= cells->dists[next_pos])
        return;
    size_t cell_idx = pf_cells_queue_get_idx(cells, next_pos);
    struct pf_cell_t *cell = heap_get(&cells->to_explore, cell_idx);
    const float dist_to_dest = cell->path_len - cells->dists[next_pos];
    cell->path_len = dist_to_dest + dist_to_cell;
    cells->dists[next_pos] = dist_to_cell;
    cells->parents[next_pos] = cur_pos;
    heap_dec_key(&cells->to_explore, cell_idx);
}

static const struct pf_cell_t* pf_cells_queue_get_closest(const struct pf_cells_t *cells){
    return heap_get_root(&cells->to_explore);
}

static void pf_cells_queue_pop_closest(struct pf_cells_t* cells){
    heap_pop_root(&cells->to_explore);
}

static bool pf_cells_queue_is_empty(struct pf_cells_t* cells){
    return heap_is_empty(&cells->to_explore);
}

#if PF_DISPLAY_SEARCH
static void pf_display_search(
    const struct terrain_map_t *map,
    struct pf_cells_t* cells,
    struct dynarr_t* path
){
    size_t npoints = terrain_map_get_npoints(map);
    char* buf = malloc(npoints + map->npoints_y + 1);
    char* head = buf;
    assert(buf);
    for (size_t y = 0; y < map->npoints_y; ++y){
        for (size_t x = 0; x < map->npoints_x; ++x){
            size_t pos = y * map->npoints_x + x;
            char c;
            if (pf_cells_tree_contains(cells, pos))
                c = '+';
            else if (pf_cells_queue_contains(cells, pos))
                c = '#';
            else
                c = '-';
            *head++ = c;
        }
        *head++ = '\n';
    }
    *head++ = '\n';
    if (path){
        for (size_t i = 0; i < path->length; ++i){
            struct point_2d_t* point = dynarr_get(path, i);
            uint32_t pos = terrain_map_get_pos(map, *point);
            pos += pos / map->npoints_x;
            buf[pos] = 'O';
        }
    }
    fwrite(buf, 1, (size_t)(head - buf), stdout);
    free(buf);
}
#endif

static bool pf_search(
    const struct terrain_map_t *map,
    struct pf_cells_t *cells,
    uint32_t cur_pos,
    const uint32_t dest_pos
) {
    cells->dists[cur_pos] = 0;
    while (true) {
        bitarr_set(cells->intree, cur_pos);
        for (uint32_t row_pos = cur_pos - 1 - map->npoints_x,
                      last_row_pos = cur_pos + map->npoints_x - 1;
             row_pos <= last_row_pos; row_pos += map->npoints_x) {
            for (uint32_t next_pos = row_pos,
                          stop_pos = row_pos + PF_NEIGHBOURHOOD_SIZE;
                 next_pos < stop_pos; ++next_pos) {
                if (!terrain_map_contains(map, next_pos))
                    continue;
                if (next_pos == cur_pos)
                    continue;
                if (pf_cells_tree_contains(cells, next_pos))
                    continue;
                if (pf_cells_queue_contains(cells, next_pos))
                    pf_cells_consider_shorter_dist(
                        map, cells, cur_pos, next_pos, dest_pos
                    );
                else
                    pf_cells_enqueue(map, cells, cur_pos, next_pos, dest_pos);
            }
        }
        if (pf_cells_tree_contains(cells, dest_pos))
            return true;
        if (pf_cells_queue_is_empty(cells)){
            return false;
        }
        const struct pf_cell_t *closest_cell = pf_cells_queue_get_closest(cells);
        if (!pf_is_reachable(closest_cell->path_len)){
            return false;
        }
        cur_pos = closest_cell->pos;
        pf_cells_queue_pop_closest(cells);
    }
}

static void pf_reverse_path(struct dynarr_t* path){
    struct point_2d_t *i = dynarr_get(path, 0);
    struct point_2d_t *j = dynarr_get(path, path->length - 1);
    for (; i < j; ++i, --j){
        struct point_2d_t temp;
        temp = *i;
        *i = *j;
        *j = temp;
    }
}

static bool pf_create_path(
    const struct terrain_map_t *map,
    uint32_t *parents,
    uint32_t start_pos,
    uint32_t cur_pos,
    struct dynarr_t *path
) {
    if (!dynarr_make(path, sizeof(struct point_2d_t), PF_PATH_INITIAL_CAPACITY))
        return false;
    parents[start_pos] = UINT32_MAX;
    for (; cur_pos != UINT32_MAX; cur_pos = parents[cur_pos]) {
        struct grid_coord_t coord = terrain_map_get_coord(map, cur_pos);
        struct point_2d_t point = terrain_map_to_point(map, coord);
        if (!dynarr_append(path, &point)){
            dynarr_free(path);
            return false;
        }
    }

    pf_reverse_path(path);
    return true;
}

#if PF_EVAL_PATH
static void pf_eval_path(
    const struct terrain_map_t *map,
    const struct pf_cells_t *cells,
    struct dynarr_t *path,
    uint32_t prev_pos,
    uint32_t dest_pos
){
    phys_dist_t total_dist_from_dest = 0;
    phys_dist_t total_dist_from_start = 0;
    phys_dist_t txy = 0;
    phys_dist_t tg = 0;
    phys_dist_t tc = 0;
    phys_dist_t twxy = 0;
    phys_dist_t twg = 0;
    phys_dist_t twc = 0;

    struct point_2d_t min_edge_point;
    struct point_2d_t max_edge_point;
    float min_edge_cost = FLT_MAX;
    float max_edge_cost = FLT_MIN;

    for (size_t i = 0; i < path->length; ++i){
        struct point_2d_t point = *(struct point_2d_t*)dynarr_get(path, i);
        uint32_t pos = terrain_map_get_pos(map, point);
        struct grid_coord_t coord = terrain_map_get_coord(map, pos);
        float edge_cost = pf_cell_calc_edge_cost(map, prev_pos, pos);
        phys_dist_t dist_to_dest = pf_cell_calc_dist(map, pos, dest_pos);
        phys_dist_t xy_dist = pf_cell_calc_xy_dist(map, prev_pos, pos);
        phys_dist_t grad = pf_cell_calc_grad(map, prev_pos, pos, xy_dist);
        phys_dist_t cost = map->data_cost[pos];
        phys_dist_t wxy_dist =  PF_WEIGHT_XY_DIST * xy_dist;
        phys_dist_t wgrad =  PF_WEIGHT_GRAD * grad;
        phys_dist_t wcost =  PF_WEIGHT_COST * cost;
        float dist = cells->dists[pos];
        total_dist_from_dest += dist_to_dest;
        total_dist_from_start += dist;
        txy += xy_dist; tg += grad; tc += cost;
        twxy += wxy_dist; twg += wgrad; twc += wcost;
        if (edge_cost < min_edge_cost){
            min_edge_point = point;
            min_edge_cost = edge_cost;
        }
        if (edge_cost > max_edge_cost) {
            max_edge_point = point;
            max_edge_cost = edge_cost;
        }
        prev_pos = pos;
        printf("%6d (%6d, %6d) (%4.1f, %4.1f) (xy=%6.2f, g=%6.2f, c=%7.2f) (wxy=%6.2f, wg=%6.2f. wc=%6.2f) (ec=%6.2f, d=%6.2f, h=%6.2f)\n",
            (int)pos,
            (int)coord.x, (int)coord.y,
            point.x, point.y,
            xy_dist, grad, cost,
            wxy_dist, wgrad, wcost,
            edge_cost, dist, dist_to_dest);
    }
    total_dist_from_start /= (float)(path->length);
    total_dist_from_dest /= (float)(path->length);
    txy /= (float)(path->length);
    tg /= (float)(path->length);
    tc /= (float)(path->length);
    twxy /= (float)(path->length);
    twg /= (float)(path->length);
    twc /= (float)(path->length);

    printf("min edge cost  : %7.2f (%4.1f, %4.1f)\n", min_edge_cost, min_edge_point.x, min_edge_point.y);
    printf("max edge cost  : %7.2f (%4.1f, %4.1f)\n", max_edge_cost, max_edge_point.x, max_edge_point.y);
    printf("avg dist strt  : %7.2f\n", total_dist_from_start);
    printf("avg dist dest  : %7.2f\n", total_dist_from_dest);
    printf("ratio src/dest : %7.2f\n", total_dist_from_start / total_dist_from_dest);
    printf("avg (xy, g, c) : (xy= %6.2f, g= %6.2f, c= %6.2f)\n", txy, tg, tc);
    printf("avg (wxy,wg,wc): (wxy=%6.2f, wg=%6.2f. wc=%6.2f)\n", twxy, twg, twc);
}
#endif

bool pathfinder(
    const struct terrain_map_t *map,
    const struct point_2d_t start,
    const struct point_2d_t dest,
    struct dynarr_t *path
) {
    struct pf_cells_t cells;
    if (!pf_cells_create(map, &cells)){
        pf_cells_free(&cells);
        return false;
    }
    const uint32_t start_pos = terrain_map_get_pos(map, start);
    const uint32_t dest_pos = terrain_map_get_pos(map, dest);
    if (!pf_search(map, &cells, start_pos, dest_pos)){
        pf_cells_free(&cells);
        return false;
    }
    const bool err = pf_create_path(map, cells.parents, start_pos, dest_pos, path);
#if PF_DISPLAY_SEARCH
    pf_display_search(map, &cells, path);
#endif
#if PF_EVAL_PATH
    pf_eval_path(map, &cells, path, start_pos, dest_pos);
#endif
    pf_cells_free(&cells);
    return err;
}
