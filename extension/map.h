#ifndef MAP_H
#define MAP_H
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "adt/dynarr.h"


/**
 * COORDINATE SYSTEM
 *
 * Map coordinate frame (REP 103)
 * When the terrain map is viewed top-down:
 *   - Origin is at the bottom left.
 *   - x-axis is horizontal along the ground plane.
 *   - y-axis is vertical along the ground plane.
 *   - z-axis is perpendicular to the ground plane.
 *   - Positive x is rightward.
 *   - Positive y is upward.
 *   - Positive z is skyward.
 *   - Positive rotation is from x to y (counterclockwise).
 *
 * Chassis (base_link) coordinate frame (REP 105)
 * From the perspective of the robot:
 *   - x-axis is along the primary direction of travel, parallel to the base
 *     plane.
 *   - y-axis is perpendicular to the primary direction of travel, parallel to
 *     the base plane.
 *   - z-axis is perpendicular to the base plane.
 *   - Positive x is forward.
 *   - Positive y is leftward.
 *   - Positive z is upward.
 *   - Roll is rotation about the x-axis. Pitch is about the y-axis. Yaw is
 *     about the z-axis. Positive rotation is determined by the right hand rule.
 *
 * Wherever an array is used to represent a grid of data, it is stored in
 * row-major (i.e., x-axis-major) order, with array indexing following the
 * coordinate conventions for the map coordinate frame.
 */


/* Denotes that this value represents a physical distance */
typedef float phys_dist_t;

/* Map of the terrain to navigate, containing multiple data layers */
struct terrain_map_t {

    // Metadata
    uint32_t npoints_x;         // Number of data points along x-axis
    uint32_t npoints_y;         // Number of data points along y-axis
    phys_dist_t resolution;     // Physical distance (e.g., meters) per point

    // Data arrays
    // All array lengths are (npoints_x * npoints_y).
    phys_dist_t *data_elevation;    // Height along z-axis
    uint8_t *data_cost;             // General cost
};

/* A 2D point, used to represent paths (as waypoints) */
struct point_2d_t {
    phys_dist_t x;
    phys_dist_t y;
};

struct grid_coord_t {
    uint32_t x;
    uint32_t y;
};

// Functions

/**
 * Allocates the data arrays for `terrain_map_t`
 *
 * The metadata fields must already be populated.
 */
extern bool terrain_map_alloc(struct terrain_map_t *map);

/* Frees the data arrays for `terrain_map_t` */
extern void terrain_map_free(struct terrain_map_t *map);

extern uint32_t terrain_map_get_npoints(const struct terrain_map_t *map);


extern bool terrain_map_contains(const struct terrain_map_t *map, uint32_t pos);

extern uint32_t terrain_map_get_pos(
    const struct terrain_map_t *map,
    const struct point_2d_t p
);


struct grid_coord_t terrain_map_get_coord(
    const struct terrain_map_t *map,
    uint32_t pos
);

struct point_2d_t terrain_map_to_point(
    const struct terrain_map_t *map,
    struct grid_coord_t coord
);

extern bool terrain_map_is_diagonal(
    const struct terrain_map_t *map,
    uint32_t cur_pos,
    uint32_t next_pos
);
#endif
