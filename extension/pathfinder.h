#ifndef PATHFINDER_H
#define PATHFINDER_H

#include "adt/dynarr.h"
#include "map.h"
#include <stdbool.h>

struct pf_extra_out_t {
    void *path_eval;
    void *search_display;
    void *visual; //visualize search
};

struct pf_weights_t {
    float wxy_dist;
    float wgrad;
    float wcost;
    float wheuristic;
    float max_grad;
};

struct pf_config_t {
    struct pf_extra_out_t extra_out;
    struct pf_weights_t weights;
    struct point_2d_t start;
    struct point_2d_t dest;
    bool smooth;
};

enum pf_result_t {
    PF_RESULT_FOUND_PATH,   // found a path from start to destination
    PF_RESULT_NO_PATH,      // could not find a path from start to destination
    PF_RESULT_ERR_MEM // failed to find a path due to a memory allocation error
};

enum {
    PF_VISUAL_UPDATE_TREE,
    PF_VISUAL_UPDATE_QUEUE,
    PF_VISUAL_UPDATE_CHANGE_EDGE
};

typedef unsigned char pf_visual_update_type_t;

struct pf_visual_update_t {
    uint32_t pos;
    uint32_t parent;
    uint32_t old_parent;
    pf_visual_update_type_t type;
};

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
extern const char *pf_result_to_str(enum pf_result_t);

extern enum pf_result_t pathfinder(
    const struct terrain_map_t *map,
    struct pf_config_t *config,
    struct dynarr_t *path
);


#endif
