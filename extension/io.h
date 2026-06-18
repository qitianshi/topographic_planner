#include "map.h"


enum io_error_t {
    ERR_OK,             // OK
    ERR_FILE,           // System file I/O error
    ERR_MALFORMED,      // Malformed file (incorrect format)
    ERR_DIMENSION,      // Incorrect dimensions (for data load)
    ERR_NPOINTS,        // Number of points loaded does not match the expected
    ERR_MEMORY          // Failed memory allocation
};


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
 * @param meta_path The metadata filepath to read (`map.config`), formatted as a
 *      .env file.
 * @param data_path The data filepath to read (`mapdata.csv`), formatted as a
 *      CSV file.
 * @param out The output pointer.
 */
enum io_error_t load_map(
    const char *meta_path,
    const char *data_path,
    struct terrain_map_t *out
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
enum io_error_t store_path(const char *out_path, const struct dynarr_t *path);
