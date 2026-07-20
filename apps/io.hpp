#pragma once

#include "pathfinder.hpp"
#include "terrainmap.hpp"
#include <string>
#include <vector>

namespace io {

enum class IoError {
    Ok,           // OK
    File,         // System file I/O error
    Malformed,    // Malformed file (incorrect format)
    Dimension,    // Incorrect dimensions (for data load)
    Npoints,      // Number of points loaded does not match the expected
    Memory,       // Failed memory allocation
    VarNotFound,  // An environment variable was not found
    VarFormat,    // invalid format for value
    VarOutOfRange // value for an environment variable was out of range
};

IoError strToUint32(const std::string &s, uint32_t &val);
IoError getenvUint32(const char *name, uint32_t &val);

const char *errorToStr(IoError err);
IoError loadConfig(pathfinder::Config &config);
void loadSmoothSteps(uint32_t &smoothSteps);

/**
 * Loads the map from a config file and a data file.
 *
 * Example `pathfinder.config` file:
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
 * of `out`, `ErrMalformed` is returned.
 *
 * @param map Output reference to the map.
 */
IoError loadMap(terrainmap::TerrainMap &map);

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
IoError storePath(const std::vector<terrainmap::Point2D> &path);

} // namespace io
