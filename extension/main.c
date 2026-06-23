#include <stdbool.h>
#include <stdio.h>

#include "io.h"
#include "map.h"
#include "pathfinder.h"
#include "smooth.h"

void report_error(enum io_error_t err) {
    fprintf(
        stderr, "pathfinder exited with error %s (%d)\n", io_err_to_str(err),
        (int)err
    );
}

// argv[1]: meta path argv[2]: data path argv[3]: out path
// argv[4]: start point (x,y) argv[5]: dest point (x,y)
int main(int argc, char **argv) {

    struct pf_config_t pf_config;
    struct terrain_map_t map;
    struct dynarr_t path;
    uint32_t smooth_steps;
    enum io_error_t err;

    io_load_smooth_steps(&smooth_steps);
    if ((err = io_load_pathfinder_config(&pf_config)) != ERR_OK) {
        report_error(err);
        return EXIT_FAILURE;
    }

    if ((err = io_load_map(&map)) != ERR_OK) {
        report_error(err);
        return EXIT_FAILURE;
    }

    const enum pf_result_t res = pathfinder(&map, &pf_config, &path);
    if (res == PF_RESULT_FOUND_PATH) {
        printf("pathfinder: %s\n", pf_result_to_str(res));
        smooth(&map, smooth_steps, &path);
        if ((err = io_store_path(&path)) != ERR_OK)
            report_error(err);
        dynarr_free(&path);
    } else {
        fprintf(stderr, "pathfinder: %s\n", pf_result_to_str(res));
    }

    terrain_map_free(&map);
    return EXIT_SUCCESS;
}
