#include <assert.h>
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
