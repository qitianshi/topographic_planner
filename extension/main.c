#include <stdio.h>
#include <stdbool.h>

#include "io.h"
#include "map.h"
#include "pathfinder.h"

void report_error(enum io_error_t err){
    fprintf(stderr, "pathfinder exited with error %s (%d)\n", io_err_to_str(err), (int)err);
}

// argv[1]: meta path argv[2]: data path argv[3]: out path
// argv[4]: start point (x,y) argv[5]: dest point (x,y)
int main(int argc, char **argv) {

    struct pf_config_t pf_config;
    struct terrain_map_t map;
    struct io_filepaths_t filepaths;
    struct dynarr_t path;

    enum io_error_t err;

    if ((err = io_load_paths(&filepaths)) != ERR_OK){
        report_error(err);
        return EXIT_FAILURE;
    }

    if ((err = io_load_pathfinder_config(&pf_config)) != ERR_OK) {
        report_error(err);
        io_free_paths(&filepaths);
        return EXIT_FAILURE;
    }

    if ((err = io_load_map(&filepaths, &map)) != ERR_OK) {
        report_error(err);
        io_free_paths(&filepaths);
        return EXIT_FAILURE;
    }

    const enum pf_result_t res = pathfinder(&map, &pf_config, &path);
    if (res == PF_RESULT_FOUND_PATH) {
        printf("pathfinder: %s\n", pf_result_to_str(res));
        if ((err = io_store_path(&filepaths, &path)) != ERR_OK)
            report_error(err);
        dynarr_free(&path);
    }

    terrain_map_free(&map);
    return EXIT_SUCCESS;
}
