#include "map.h"
#include "pathfinder.h"


enum io_error_t {
    ERR_OK,              // OK
    ERR_FILE,            // System file I/O error
    ERR_MALFORMED,       // Malformed file (incorrect format)
    ERR_DIMENSION,       // Incorrect dimensions (for data load)
    ERR_NPOINTS,         // Number of points loaded does not match the expected
    ERR_MEMORY,          // Failed memory allocation
    ERR_VAR_NOT_FOUND,   // An environment variable was not found
    ERR_VAR_FORMAT,      // invalid format for value
    ERR_VAR_OUT_OF_RANGE // value for an environment variable was out of range
};

struct io_filepaths_t {
    char *mapdata;    //csv map data input file
    char *pathout;    //csv path data output file
};

extern const char* io_err_to_str(enum io_error_t err);
extern enum io_error_t io_load_paths(struct io_filepaths_t* paths);
extern void io_free_paths(struct io_filepaths_t* paths);
extern enum io_error_t io_load_pathfinder_config(struct pf_config_t* config);


/**
 * Loads the map from a config file and a data file.
 *
 * Example `map.config` file:
 *
 * ```
 * NPOINTS_X=5000
 * NPOINTS_Y=10000
 * RESOLUTION=1.0
 * ```
 *
 * Example `mapdata.csv` file:
 *
 * ```
 * x,y,elevation,cost
 * 0,0,1.0,255
 * 0,1,1.5,250
 * 0,2,1.2,0
 * ```
 *
 * The CSV file must be in row-major (x-axis-major) order. The `x` and `y`
 * values are integers corresponding to the index of the data point (not
 * physical coordinates).
 *
 * If the number and indexes of points do not match `npoints_x` and `npoints_y`
 * of `out`, `ERR_MALFORMED` is returned.
 *
 * @param paths The data filepath to read (`mapdata.csv`), formatted as a
 *      CSV file.
 * @param map Output pointer to the map.
 */
extern enum io_error_t io_load_map(
    struct io_filepaths_t* paths,
    struct terrain_map_t *map
);

/**
 * Stores a terrain path to an output file.
 *
 * Example `path.csv` output file:
 *
 * ```
 * x,y
 * 0.1,0.2
 * 0.3,0.4
 * ```
 */
extern enum io_error_t io_store_path(struct io_filepaths_t* paths, const struct dynarr_t *path);
