#include <assert.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "map.h"


bool terrain_map_alloc(struct terrain_map_t *map) {

    assert(map);

    // Cast to size_t for dynarr
    size_t num_elements = (size_t)map->npoints_x * (size_t)map->npoints_y;

    // Guards against 0 case by failing
    if (num_elements == 0) {
        map->data_elevation = NULL;
        map->data_cost = NULL;
        return false;
    }

    map->data_elevation =
        malloc(num_elements * sizeof(*map->data_elevation));
    if (!map->data_elevation) {
        return false;
    }

    map->data_cost = malloc(num_elements * sizeof(*map->data_cost));
    if (!map->data_cost) {
        // If second malloc fails, free the first to prevent memory leaks
        free(map->data_elevation);
        map->data_elevation = NULL;
        return false;
    }

    return true;
}

/* Frees the data arrays for `terrain_map_t` */
void terrain_map_free(struct terrain_map_t *map) {

    // No-op if map is NULL
    if (!map) {
        return;
    }

    free(map->data_elevation);
    free(map->data_cost);

    // Resets pointers to prevent use after free errors
    map->data_elevation = NULL;
    map->data_cost = NULL;
}

uint32_t terrain_map_get_npoints(const struct terrain_map_t *map){
    return map->npoints_x * map->npoints_y;
}


bool terrain_map_contains(
    const struct terrain_map_t *map, uint32_t pos
) {
    return pos < terrain_map_get_npoints(map);
}


uint32_t terrain_map_get_pos(
    const struct terrain_map_t *map,
    const struct point_2d_t p
) {
    const uint32_t cell_x = (uint32_t)(p.x / map->resolution);
    const uint32_t cell_y = (uint32_t)(p.y / map->resolution);
    return cell_y * map->npoints_x + cell_x;
}


struct grid_coord_t terrain_map_get_coord(
    const struct terrain_map_t *map,
    uint32_t pos
) {
    assert(pos < LONG_MAX);
    ldiv_t div = ldiv(pos, map->npoints_x);
    return (struct grid_coord_t){.y = (uint32_t)div.quot, .x = (uint32_t)div.rem};
}

struct point_2d_t terrain_map_to_point(
    const struct terrain_map_t *map,
    struct grid_coord_t coord
){
    return (struct point_2d_t){
        .x = (phys_dist_t)coord.x * map->resolution,
        .y = (phys_dist_t)coord.y * map->resolution
    };
}

bool terrain_map_is_diagonal(
    const struct terrain_map_t *map,
    uint32_t cur_pos,
    uint32_t next_pos
) {
    struct grid_coord_t cur_coord = terrain_map_get_coord(map, cur_pos);
    struct grid_coord_t next_coord = terrain_map_get_coord(map, next_pos);
    return cur_coord.x != next_coord.x && cur_coord.y != next_coord.y;
}
