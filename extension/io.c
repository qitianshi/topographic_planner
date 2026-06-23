#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "smooth.h"

#include "io.h"
#include "pathfinder.h"

#define MAX_META_LINE_LEN 64
#define MAX_META_LABEL_LEN 32
#define MAX_META_VALUE_LEN ((MAX_META_LINE_LEN) - (MAX_META_LABEL_LEN) - 1)
#define CONFIG_LINE_FORMAT "%31[^=]=%31s"
#define EXPECTED_MATCHES 4
#define EXPECTED_POINT_INPUT_NUM 2

extern const char *io_err_to_str(enum io_error_t err) {
    const char *err_msg[] = {
        [ERR_OK] = "ERR_OK: no errors",
        [ERR_FILE] = "ERR_FILE: System file I/O error",
        [ERR_MALFORMED] = "ERR_MALFORMED: Malformed file (incorrect format)",
        [ERR_DIMENSION] = "ERR_DIMENSION: Incorrect dimensions (for data load)",
        [ERR_NPOINTS] =
            "ERR_NPOINTS: Number of points loaded does not match the expected",
        [ERR_MEMORY] = "ERR_MEMORY: Failed memory allocation",
        [ERR_VAR_NOT_FOUND] =
            "ERR_VAR_NOT_FOUND: An environment variable was not found",
        [ERR_VAR_FORMAT] = "ERR_VAR_FORMAT: invalid format for value",
        [ERR_VAR_OUT_OF_RANGE] = "ERR_VAR_OUT_OF_RANGE: value for an "
                                 "environment variable was out of range"
    };
    if (err < ERR_OK || err > ERR_VAR_OUT_OF_RANGE)
        return "Unknown Error";
    return err_msg[err];
}

extern enum io_error_t str_to_uint32(const char *s, uint32_t *val) {
    errno = 0;
    unsigned long ulong_val;
    char *end;
    ulong_val = strtoul(s, &end, 10);
    if (end == s)
        return ERR_VAR_FORMAT;
    else if (errno == ERANGE || ulong_val > UINT32_MAX)
        return ERR_VAR_OUT_OF_RANGE;
    else {
        *val = (uint32_t)ulong_val;
        return ERR_OK;
    }
}

static enum io_error_t str_to_float(const char *s, float *val) {
    errno = 0;
    char *end;
    *val = strtof(s, &end);
    if (end == s)
        return ERR_VAR_FORMAT;
    else if (errno == ERANGE)
        return ERR_VAR_OUT_OF_RANGE;
    else
        return ERR_OK;
}


// Checks point is in correct format and updates struct values
static enum io_error_t str_to_point(const char *s, struct point_2d_t *point) {
    if (sscanf(s, " (%f,%f)", &(point->x), &(point->y)) !=
        EXPECTED_POINT_INPUT_NUM)
        return ERR_VAR_FORMAT;
    else
        return ERR_OK;
}

static enum io_error_t str_to_bool(const char* s, bool* b){
    uint32_t val;
    enum io_error_t err = str_to_uint32(s, &val);
    if (err != ERR_VAR_FORMAT){
        *b = val;
        return ERR_OK;
    }
    char cpy[6];
    int i;
    for (i = 0; i < 5 && s[i]; ++i)
        cpy[i] = (char)tolower((unsigned char)s[i]);
    cpy[i] = '\0';
    if (i == 4 && !strcmp("true", cpy))
        *b = true;
    else if (i == 5 && !strcmp("false", cpy))
        *b = false;
    else
        return ERR_VAR_FORMAT;
    return ERR_OK;
}

/* Copy implementation of POSIX strdup */
static char *pf_strdup(const char *s) {

    if (!s)
        return NULL;

    size_t len = strlen(s);

    // +1 byte for the '\0' terminator
    char *dup = malloc(len + 1);
    if (!dup)
        return NULL;

    memcpy(dup, s, len + 1);
    return dup;
}


extern enum io_error_t getenv_uint32(const char *name, uint32_t *val) {
    const char *valstr = getenv(name);
    return valstr ? str_to_uint32(valstr, val) : ERR_VAR_NOT_FOUND;
}

static enum io_error_t getenv_float(const char *name, float *val) {
    const char *valstr = getenv(name);
    return valstr ? str_to_float(valstr, val) : ERR_VAR_NOT_FOUND;
}

static enum io_error_t getenv_point(
    const char *name,
    struct point_2d_t *point
) {
    const char *valstr = getenv(name);
    return valstr ? str_to_point(valstr, point) : ERR_VAR_NOT_FOUND;
}

static enum io_error_t getenv_str(const char *name, char **val) {
    const char *valstr = getenv(name);
    if (!valstr)
        return ERR_VAR_NOT_FOUND;
    *val = pf_strdup(valstr);
    return *val ? ERR_OK : ERR_MEMORY;
}


extern enum io_error_t getenv_bool(const char* name, bool* b){
    const char* valstr = getenv(name);
    return valstr? str_to_bool(valstr, b) : ERR_VAR_NOT_FOUND;
}

/**
 * Sets the metadata of the struct from the config file
 * @param meta_path The file to read from
 * @param out The struct to edit
 */
static enum io_error_t load_map_meta(struct terrain_map_t *map) {
    enum io_error_t err;
    if ((err = getenv_uint32("NPOINTS_X", &map->npoints_x)) != ERR_OK) {
        fputs("failed to retrieve environment variable NPOINTS_X\n", stderr);
        return err;
    }
    if ((err = getenv_uint32("NPOINTS_Y", &map->npoints_y)) != ERR_OK) {
        fputs("failed to retrieve environment variable NPOINTS_Y\n", stderr);
        return err;
    }
    if ((err = getenv_float("RESOLUTION", &map->resolution)) != ERR_OK) {
        fputs("failed to retrieve environment variable RESOLUTION\n", stderr);
        return err;
    }
    return ERR_OK;
}

static enum io_error_t load_start_dest(
    struct point_2d_t *start,
    struct point_2d_t *dest
) {
    enum io_error_t err;
    if ((err = getenv_point("START", start)) != ERR_OK) {
        fputs("failed to retrieve environment variable START\n", stderr);
        return err;
    }
    if ((err = getenv_point("DEST", dest)) != ERR_OK) {
        fputs("failed to retrieve environment variable DEST\n", stderr);
        return err;
    }
    return ERR_OK;
}

static enum io_error_t load_weights(struct pf_weights_t *weights) {
    enum io_error_t err;
    if ((err = getenv_float("PF_WEIGHT_XY_DIST", &weights->wxy_dist)) !=
        ERR_OK) {
        fputs(
            "failed to retrieve environment variable PF_WEIGHT_XY_DIST\n",
            stderr
        );
        return err;
    }
    if ((err = getenv_float("PF_WEIGHT_GRAD", &weights->wgrad)) != ERR_OK) {
        fputs(
            "failed to retrieve environment variable PF_WEIGHT_GRAD\n", stderr
        );
        return err;
    }
    if ((err = getenv_float("PF_WEIGHT_COST", &weights->wcost)) != ERR_OK) {
        fputs(
            "failed to retrieve environment variable PF_WEIGHT_COST\n", stderr
        );
        return err;
    }
    if ((err = getenv_float("PF_WEIGHT_HEURISTIC", &weights->wheuristic)) !=
        ERR_OK) {
        fputs(
            "failed to retrieve environment variable PF_WEIGHT_HEURISTIC\n",
            stderr
        );
        return err;
    }
    if ((err = getenv_float("PF_MAX_GRADIENT", &weights->max_grad)) != ERR_OK) {
        fputs(
            "failed to retrieve environment variable PF_MAX_GRADIENT\n", stderr
        );
        return err;
    }
    return ERR_OK;
}

static void *load_extra_out_config(const char *name) {
    name = getenv(name);
    if (!name)
        return NULL;
    if (!strcmp("NULL", name) || !strcmp("null", name))
        return NULL;
    if (!strcmp("stdout", name))
        return stdout;
    FILE *f = fopen(name, "w");
    if (!f)
        return NULL;
    if (fputc('\n', f) == EOF) {
        fclose(f);
        return NULL;
    }
    fclose(f);
    return pf_strdup(name);
}

static void load_extra_out_configs(struct pf_extra_out_t *extra_out) {
    extra_out->path_eval = load_extra_out_config("PF_EVAL_PATH");
    extra_out->search_display = load_extra_out_config("PF_DISPLAY_SEARCH");
    char* visualization;
    getenv_str("PF_VISUALIZE_SEARCH", &visualization);
    extra_out->visual = visualization;
}

extern enum io_error_t io_load_pathfinder_config(struct pf_config_t *config) {
    enum io_error_t err;
    if ((err = load_start_dest(&config->start, &config->dest)) != ERR_OK)
        return err;
    if ((err = load_weights(&config->weights)) != ERR_OK)
        return err;
    load_extra_out_configs(&config->extra_out);
    return ERR_OK;
}

extern void io_load_smooth_steps(uint32_t* smooth_steps){
    if (getenv_uint32("SMOOTH_STEPS", smooth_steps) != ERR_OK)
        *smooth_steps = SMOOTH_STEPS_DEFAULT;
}

/**
 * Sets the data of the file from the csv file
 * @param meta_path The file to read from
 * @param out The struct to edit
 */
static enum io_error_t load_map_data(
    const char *data_path,
    struct terrain_map_t *out
) {
    FILE *data_file = fopen(data_path, "r");
    if (data_file == NULL) {
        return ERR_FILE;
    }

    // Initialise variables for fscanf later
    int x = 0;
    int y = 0;
    phys_dist_t elevation = 0;
    uint8_t cost = 0;

    int x_next = 0;
    int y_next = 0;

    int index = 0;

    // Skip header of csv file
    if (fscanf(data_file, "%*[^\n]\n") == EOF) {
        fclose(data_file);
        return ERR_FILE;
    }

    // Check the validity of each line of data and set the appropriate values
    for (int matches;
         (matches = fscanf(
              data_file, "%d,%d,%f,%" SCNu8, &x, &y, &elevation, &cost
          )) != EOF;) {
        if (matches != EXPECTED_MATCHES) {
            fprintf(
                stderr, "%s: malformed input on line %d\n", __func__, index + 1
            );
            fclose(data_file);
            return ERR_MALFORMED;
        }
        // Check x and y values are valid
        if (x != x_next || y != y_next || x >= out->npoints_x ||
            y >= out->npoints_y) {
            fclose(data_file);
            return ERR_DIMENSION;
        }

        // Update values
        if (x >= out->npoints_x - 1) {
            x_next = 0;
            ++y_next;
        } else {
            ++x_next;
        }
        out->data_elevation[index] = elevation;
        out->data_cost[index] = cost;
        index++;
    }
    enum io_error_t err = ERR_OK;
    if (y_next != out->npoints_y) {
        err = feof(data_file) ? ERR_FILE : ERR_NPOINTS;
    }
    if (fclose(data_file) != 0) {
        err = ERR_FILE;
    }

    return err;
}

// Load data from input files to out
extern enum io_error_t io_load_map(
    struct terrain_map_t *map
) {
    enum io_error_t err;
    while (true) {
        // Must set metadata values of out before data values
        if ((err = load_map_meta(map)) != ERR_OK)
            break;
        if (!terrain_map_alloc(map)) {
            fputs("malloc for struct terrain_map_t *out failed\n", stderr);
            err = ERR_MEMORY;
            break;
        }
        const char* mapdata = getenv("MAPDATA");
        if (!mapdata){
            fputs("failed to retrieve environment variable MAPDATA\n", stderr);
            err = ERR_VAR_NOT_FOUND;
            break;
        }
        if ((err = load_map_data(mapdata, map)) != ERR_OK) {
            terrain_map_free(map);
            fprintf(stderr, "failed to load map data from %s\n", mapdata);
        }
        break;
    }
    return err;
}

static enum io_error_t write_path(FILE *out, const struct dynarr_t *path) {
    // Print header to csv file
    fprintf(out, "x,y\n");
    if (ferror(out))
        return ERR_FILE;
    // Print all x and y values in path to output file
    for (size_t i = 0; i < path->length; i++) {
        const struct point_2d_t *point = dynarr_get(path, i);
        fprintf(out, "%f,%f\n", point->x, point->y);
        if (ferror(out))
            return ERR_FILE;
    }
    return ERR_OK;
}

// Store data from out into csv output file
extern enum io_error_t io_store_path(
    const struct dynarr_t *path
) {
    const char* pathout = getenv("PATHOUT");
    if (!pathout){
        fputs("failed to retrieve environment variable PATHOUT\n", stderr);
        return ERR_VAR_NOT_FOUND;
    }
    FILE *out_file = fopen(pathout, "w");
    enum io_error_t err;
    if (out_file) {
        if ((err = write_path(out_file, path)) != ERR_OK)
            fprintf(
                stderr, "write error while storing generated path in file %s\n",
                pathout
            );
        if (fclose(out_file) != 0) {
            fprintf(stderr, "failed to close file %s\n", pathout);
            err = ERR_FILE;
        }
    } else {
        fprintf(stderr, "failed to open path output file %s\n", pathout);
        err = ERR_FILE;
    }
    return err;
}
