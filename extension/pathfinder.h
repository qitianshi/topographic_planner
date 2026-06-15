#include <stdbool.h>

#include "adt/dynarr.h"
#include "map.h"


/**
 * Calculates the optimal path in the `map` from `start` to `dest`
 *
 * @param map The map representing the terrain to traverse.
 * @param start The start point.
 * @param dest The destination point.
 * @param path The output parameter, a `dynarr_t` of `point_2d_t` representing
 *      an ordered list of waypoints.
 * @return `true` if a path was successfully found, `false` otherwise.
 */
bool pathfinder(
    const struct terrain_map_t *map,
    const struct point_2d_t *start,
    const struct point_2d_t *dest,
    struct dynarr_t *path
);
