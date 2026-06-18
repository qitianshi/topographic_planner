#include <stdio.h>
#include <stdbool.h>

#include "io.h"
#include "map.h"
#include "pathfinder.h"
#include "smooth.h"

#define META_PATH_IDX 1
#define DATA_PATH_IDX 2
#define OUTPUT_PATH_IDX 3
#define START_POINT_IDX 4
#define DEST_POINT_IDX 5

#define EXPECTED_POINT_INPUT_NUM 2



// Checks point is in correct format and updates struct values
static bool parse_point(struct point_2d_t *point, char *input) {
    if (
        sscanf(input, "(%f,%f)", &(point->x), &(point->y))
        != EXPECTED_POINT_INPUT_NUM
    ) {
        return false;
    }

    return true;
}

// argv[1]: meta path argv[2]: data path argv[3]: out path
// argv[4]: start point (x,y) argv[5]: dest point (x,y)
int main(int argc, char **argv) {

    struct point_2d_t start;
    struct point_2d_t dest;

    // Set start and dest x and y values from input
    if (!parse_point(&start, argv[START_POINT_IDX])) {
        fprintf(stderr, "Invalid format for start point input, expected (x,y)");
        return EXIT_FAILURE;
    }

    if (!parse_point(&dest, argv[DEST_POINT_IDX])) {
        fprintf(stderr, "Invalid format for dest point input, expected (x,y)");
        return EXIT_FAILURE;
    }

    struct terrain_map_t map;

    // Read metadata and data from inputs and write to map
    enum io_error_t err = load_map(
        argv[META_PATH_IDX],
        argv[DATA_PATH_IDX],
        &map
    );
    if (err != ERR_OK) {
        fprintf(stderr, "load_map exited with error %d\n", (int)err);
        return EXIT_FAILURE;
    }

    struct dynarr_t path;

    if (!pathfinder(&map, start, dest, &path)) {
        return EXIT_FAILURE;
    }

    smooth(&map,&path);

    // Write path to output file
    err = store_path(
        argv[OUTPUT_PATH_IDX],
        &path
    );
    if (err != ERR_OK) {
        fprintf(stderr, "store_path exited with error %d\n", (int)err);
        return EXIT_FAILURE;
    }

    terrain_map_free(&map);

    return EXIT_SUCCESS;
}
