#include "io.hpp"
#include "smooth.hpp"
#include "pathfinder.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <cinttypes>
#include <algorithm>
#include <cctype>

namespace io {

const char* errorToStr(IoError err) {
    switch (err) {
        case IoError::Ok: return "IoError::Ok: no errors";
        case IoError::File: return "IoError::File: System file I/O error";
        case IoError::Malformed: return "IoError::Malformed: Malformed file (incorrect format)";
        case IoError::Dimension: return "IoError::Dimension: Incorrect dimensions (for data load)";
        case IoError::Npoints: return "IoError::Npoints: Number of points loaded does not match the expected";
        case IoError::Memory: return "IoError::Memory: Failed memory allocation";
        case IoError::VarNotFound: return "IoError::VarNotFound: An environment variable was not found";
        case IoError::VarFormat: return "IoError::VarFormat: invalid format for value";
        case IoError::VarOutOfRange: return "IoError::VarOutOfRange: value for an environment variable was out of range";
        default: return "Unknown Error";
    }
}

IoError strToUint32(const std::string& s, uint32_t& val) {
    if (s.empty()) return IoError::VarFormat;
    try {
        size_t pos = 0;
        unsigned long ulongVal = std::stoul(s, &pos, 10);
        if (pos == 0) return IoError::VarFormat;
        if (ulongVal > UINT32_MAX) return IoError::VarOutOfRange;
        val = static_cast<uint32_t>(ulongVal);
        return IoError::Ok;
    } catch (const std::invalid_argument&) {
        return IoError::VarFormat;
    } catch (const std::out_of_range&) {
        return IoError::VarOutOfRange;
    }
}

static IoError strToFloat(const std::string& s, float& val) {
    if (s.empty()) return IoError::VarFormat;
    try {
        size_t pos = 0;
        val = std::stof(s, &pos);
        if (pos == 0) return IoError::VarFormat;
        return IoError::Ok;
    } catch (const std::invalid_argument&) {
        return IoError::VarFormat;
    } catch (const std::out_of_range&) {
        return IoError::VarOutOfRange;
    }
}

static IoError strToPoint(const std::string& s, terrainmap::Point2D& point) {
    if (sscanf(s.c_str(), " (%f,%f)", &point.x, &point.y) != 2) {
        return IoError::VarFormat;
    }
    return IoError::Ok;
}

static IoError strToBool(const std::string& s, bool& b) {
    uint32_t val;
    if (strToUint32(s, val) == IoError::Ok) {
        b = (val != 0);
        return IoError::Ok;
    }

    std::string lower = s;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c){ return std::tolower(c); });

    if (lower == "true") {
        b = true;
        return IoError::Ok;
    } else if (lower == "false") {
        b = false;
        return IoError::Ok;
    }
    return IoError::VarFormat;
}

IoError getenvUint32(const char* name, uint32_t& val) {
    const char* valstr = std::getenv(name);
    return valstr ? strToUint32(valstr, val) : IoError::VarNotFound;
}

static IoError getenvFloat(const char* name, float& val) {
    const char* valstr = std::getenv(name);
    return valstr ? strToFloat(valstr, val) : IoError::VarNotFound;
}

static IoError getenvPoint(const char* name, terrainmap::Point2D& point) {
    const char* valstr = std::getenv(name);
    return valstr ? strToPoint(valstr, point) : IoError::VarNotFound;
}

static IoError getenvStr(const char* name, std::string& val) {
    const char* valstr = std::getenv(name);
    if (!valstr) return IoError::VarNotFound;
    val = valstr;
    return IoError::Ok;
}

static IoError getenvBool(const char* name, bool& b) {
    const char* valstr = std::getenv(name);
    return valstr ? strToBool(valstr, b) : IoError::VarNotFound;
}

static IoError loadMapMeta(terrainmap::TerrainMap& map) {
    IoError err;
    if ((err = getenvUint32("NPOINTS_X", map.npointsX)) != IoError::Ok) {
        std::cerr << "failed to retrieve environment variable NPOINTS_X\n";
        return err;
    }
    if ((err = getenvUint32("NPOINTS_Y", map.npointsY)) != IoError::Ok) {
        std::cerr << "failed to retrieve environment variable NPOINTS_Y\n";
        return err;
    }
    if ((err = getenvFloat("RESOLUTION", map.resolution)) != IoError::Ok) {
        std::cerr << "failed to retrieve environment variable RESOLUTION\n";
        return err;
    }
    return IoError::Ok;
}

static IoError loadStartDest(terrainmap::Point2D& start, terrainmap::Point2D& dest) {
    IoError err;
    if ((err = getenvPoint("START", start)) != IoError::Ok) {
        std::cerr << "failed to retrieve environment variable START\n";
        return err;
    }
    if ((err = getenvPoint("DEST", dest)) != IoError::Ok) {
        std::cerr << "failed to retrieve environment variable DEST\n";
        return err;
    }
    return IoError::Ok;
}

static IoError loadWeights(pathfinder::Weights& weights) {
    IoError err;
    if ((err = getenvFloat("PF_WEIGHT_XY_DIST", weights.wxyDist)) != IoError::Ok) {
        std::cerr << "failed to retrieve environment variable PF_WEIGHT_XY_DIST\n";
        return err;
    }
    if ((err = getenvFloat("PF_WEIGHT_GRAD", weights.wgrad)) != IoError::Ok) {
        std::cerr << "failed to retrieve environment variable PF_WEIGHT_GRAD\n";
        return err;
    }
    if ((err = getenvFloat("PF_WEIGHT_COST", weights.wcost)) != IoError::Ok) {
        std::cerr << "failed to retrieve environment variable PF_WEIGHT_COST\n";
        return err;
    }
    if ((err = getenvFloat("PF_WEIGHT_HEURISTIC", weights.wheuristic)) != IoError::Ok) {
        std::cerr << "failed to retrieve environment variable PF_WEIGHT_HEURISTIC\n";
        return err;
    }
    if ((err = getenvFloat("PF_MAX_GRADIENT", weights.maxGrad)) != IoError::Ok) {
        std::cerr << "failed to retrieve environment variable PF_MAX_GRADIENT\n";
        return err;
    }
    return IoError::Ok;
}

static FILE* loadExtraOutConfig(const char* name) {
    const char* envName = std::getenv(name);
    if (!envName) return nullptr;
    std::string s(envName);
    if (s == "NULL" || s == "null") return nullptr;
    if (s == "stdout") return stdout;

    FILE* f = fopen(envName, "w");
    if (!f) return nullptr;
    if (fputc('\n', f) == EOF) {
        fclose(f);
        return nullptr;
    }
    fclose(f);

    f = fopen(envName, "a");
    return f;
}

static void loadExtraOutConfigs(pathfinder::ExtraOut& extraOut) {
    extraOut.pathEval = loadExtraOutConfig("PF_EVAL_PATH");
    extraOut.searchDisplay = loadExtraOutConfig("PF_DISPLAY_SEARCH");

    const char* vis = std::getenv("PF_VISUALIZE_SEARCH");
    if (vis && std::string(vis) != "NULL" && std::string(vis) != "null") {
        extraOut.visual = fopen(vis, "wb");
    } else {
        extraOut.visual = nullptr;
    }
}

IoError loadConfig(pathfinder::Config& config) {
    IoError err;
    if ((err = loadStartDest(config.start, config.dest)) != IoError::Ok) return err;
    if ((err = loadWeights(config.weights)) != IoError::Ok) return err;
    loadExtraOutConfigs(config.extraOut);
    return IoError::Ok;
}

void loadSmoothSteps(uint32_t& smoothSteps) {
    if (getenvUint32("SMOOTH_STEPS", smoothSteps) != IoError::Ok) {
        smoothSteps = SMOOTH_STEPS_DEFAULT;
    }
}

static IoError loadMapData(const char* dataPath, terrainmap::TerrainMap& out) {
    std::ifstream file(dataPath);
    if (!file.is_open()) return IoError::File;

    std::string line;
    // Skip header
    if (!std::getline(file, line)) return IoError::File;

    uint32_t expectedX = 0;
    uint32_t expectedY = 0;
    size_t index = 0;
    size_t maxIndex = static_cast<size_t>(out.npointsX) * static_cast<size_t>(out.npointsY);

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        uint32_t x, y;
        terrainmap::PhysDist elevation;
        uint32_t costInt; // read as int, then cast to uint8_t

        for (char& c : line) {
            if (c == ',') c = ' ';
        }

        std::istringstream iss(line);
        if (!(iss >> x >> y >> elevation >> costInt)) {
            std::cerr << "loadMapData: malformed input on line " << (index + 2) << "\n";
            return IoError::Malformed;
        }

        if (x != expectedX || y != expectedY || x >= out.npointsX || y >= out.npointsY) {
            return IoError::Dimension;
        }

        if (x >= out.npointsX - 1) {
            expectedX = 0;
            ++expectedY;
        } else {
            ++expectedX;
        }

        if (index < maxIndex) {
            out.dataElevation[index] = elevation;
            out.dataCost[index] = static_cast<uint8_t>(costInt);
            ++index;
        }
    }

    if (expectedY != out.npointsY) {
        return IoError::Npoints;
    }

    return IoError::Ok;
}

IoError loadMap(terrainmap::TerrainMap& map) {
    IoError err;
    if ((err = loadMapMeta(map)) != IoError::Ok) return err;
    if (!map.alloc()) {
        std::cerr << "allocation for TerrainMap failed\n";
        return IoError::Memory;
    }

    const char* mapdata = std::getenv("MAPDATA");
    if (!mapdata) {
        std::cerr << "failed to retrieve environment variable MAPDATA\n";
        map.free();
        return IoError::VarNotFound;
    }

    if ((err = loadMapData(mapdata, map)) != IoError::Ok) {
        std::cerr << "failed to load map data from " << mapdata << "\n";
        map.free();
    }
    return err;
}

IoError storePath(const std::vector<terrainmap::Point2D>& path) {
    const char* pathout = std::getenv("PATHOUT");
    if (!pathout) {
        std::cerr << "failed to retrieve environment variable PATHOUT\n";
        return IoError::VarNotFound;
    }

    std::ofstream out(pathout);
    if (!out) {
        std::cerr << "failed to open path output file " << pathout << "\n";
        return IoError::File;
    }

    out << "x,y\n";
    if (out.fail()) return IoError::File;

    for (const auto& point : path) {
        out << point.x << "," << point.y << "\n";
        if (out.fail()) return IoError::File;
    }

    out.close();
    if (out.fail()) {
        std::cerr << "failed to close file " << pathout << "\n";
        return IoError::File;
    }

    return IoError::Ok;
}

}
