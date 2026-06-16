#include <stdint.h>
#include <stdio.h>

#include "io.h"
#include "map.h"

#define MAX_META_LINE_LEN 64
#define MAX_META_LABEL_LEN 32
#define MAX_META_VALUE_LEN ((MAX_META_LINE_LEN) - (MAX_META_LABEL_LEN) - 1)
#define CONFIG_LINE_FORMAT "%31[^=]=%31s"

/**
 * Sets the metadata of the struct from the config file
 * @param meta_path The file to read from
 * @param out The struct to edit
 */
static enum io_error_t load_map_meta(
    const char *meta_path,
    struct terrain_map_t *out
) {
    FILE *meta_file = fopen(meta_path, "r");
    if (meta_file == NULL) {
        return ERR_FILE;
    }

    char curr_line[MAX_META_LINE_LEN];
    char label[MAX_META_LABEL_LEN];
    char value[MAX_META_VALUE_LEN];

    // Check the section of each line up to the '=' character, and then
    // set the appropriate values
    while (fgets(curr_line, MAX_META_LINE_LEN, meta_file) != NULL) {
        if (sscanf(curr_line, CONFIG_LINE_FORMAT, label, &value) == 2) {
            if (strcmp(label, "NPOINTS_X") == 0) {
                out->npoints_x = (int)strtol(value, NULL, 10);
            } else if (strcmp(label, "NPOINTS_Y") == 0) {
                out->npoints_y = (int)strtol(value, NULL, 10);
            } else if (strcmp(label, "RESOLUTION") == 0) {
                out->resolution = strtof(value, NULL);
            } else {
                // Invalid line detected
                return ERR_MALFORMED;
            }
        } else {
            // Invalid line detected
            return ERR_MALFORMED;
        }
    }

    if (fclose(meta_file) != 0) {
        return ERR_FILE;
    }

    return ERR_OK;
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
    fscanf(data_file, "%*1[\n]");

    // Check the validity of each line of data and set the appropriate values
    while (
        fscanf(data_file, "%d,%d,%f,%c", &x, &y, &elevation, &cost) != EOF
    ) {
        // Check x and y values are valid
        if (
            x != x_next ||
            y != y_next ||
            x >= out->npoints_x ||
            y >= out->npoints_y
        ) {
            return ERR_DIMENSION;
        }

        // Update values
        if (y >= out->npoints_y - 1) {
            x_next++;
            y_next = 0;
        } else {
            y_next++;
        }
        out->data_elevation[index] = elevation;
        out->data_cost[index] = cost;
        index++;
    }

    if (fclose(data_file) != 0) {
        return ERR_FILE;
    }

    return ERR_OK;
}

// Load data from input files to out
enum io_error_t load_map(
    char *meta_path,
    char *data_path,
    struct terrain_map_t *out
) {
    // Must set metadata values of out before data values
    enum io_error_t err = load_map_meta(meta_path, out);
    if (err != ERR_OK) {
        return err;
    }

    if (!terrain_map_alloc(out)) {
        fprintf(stderr, "malloc for struct terrain_map_t *out failed");
        return ERR_MEMORY;
    }

    err = load_map_data(data_path, out);
    if (err != ERR_OK) {
        return err;
    }

    return ERR_OK;
}

// Store data from out into csv output file
enum io_error_t store_path(const char *out_path, const struct dynarr_t *path) {

    FILE *out_file = fopen(out_path, "w");
    if (out_file == NULL) {
        return ERR_FILE;
    }

    // Print header to csv file
    fprintf(out_file, "x,y");

    // Print all x and y values in path to output file
    for (int i = 0; i < path->length; i++) {
        struct point_2d_t *point = dynarr_get(path, i);
        fprintf(out_file, "%f,%f\n", point->x, point->y);
    }

    if (fclose(out_file) != 0) {
        return ERR_FILE;
    }

    return ERR_OK;
}
