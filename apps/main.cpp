#include <cstdlib>
#include <iostream>
#include <vector>

#include "io.hpp"
#include "pathfinder.hpp"
#include "smooth.hpp"
#include "terrainmap.hpp"

void reportError(io::IoError err) {
    std::cerr << "pathfinder exited with error " << io::errorToStr(err) << " ("
              << static_cast<int>(err) << ")\n";
}

int main(int argc, char **argv) {
    pathfinder::Config pfConfig;
    terrainmap::TerrainMap map;
    std::vector<terrainmap::Point2D> path;
    uint32_t smoothSteps;
    io::IoError err;

    io::loadSmoothSteps(smoothSteps);

    if ((err = io::loadConfig(pfConfig)) != io::IoError::Ok) {
        reportError(err);
        return EXIT_FAILURE;
    }

    if ((err = io::loadMap(map)) != io::IoError::Ok) {
        reportError(err);
        return EXIT_FAILURE;
    }

    const pathfinder::Result res = pathfinder::search(map, pfConfig, path);
    if (res == pathfinder::Result::FoundPath) {
        std::cout << "pathfinder: " << pathfinder::resultToStr(res) << "\n";
        smooth::apply(map, smoothSteps, path);
        if ((err = io::storePath(path)) != io::IoError::Ok) {
            reportError(err);
        }
    } else {
        std::cerr << "pathfinder: " << pathfinder::resultToStr(res) << "\n";
    }

    // map destructor will automatically free vectors, no manual free needed.

    // Also clean up visual files if any
    if (pfConfig.extraOut.visual) {
        fclose(pfConfig.extraOut.visual);
    }
    if (pfConfig.extraOut.pathEval) {
        fclose(pfConfig.extraOut.pathEval);
    }
    if (pfConfig.extraOut.searchDisplay) {
        fclose(pfConfig.extraOut.searchDisplay);
    }

    return EXIT_SUCCESS;
}
