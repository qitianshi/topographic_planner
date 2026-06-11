#include <stdint.h>
#include <stdio.h>

#include "io.h"
#include "map.h"

#define MAX_LINE_LEN 64

#define MAX_LABEL_LEN 32

#define CONFIG_LINE_FORMAT "%31[^=]=%s"

// Sets the metadata of the struct from the config file
static enum io_error_t load_map_meta(
    const char *meta_path,                // The file to read from
    struct terrain_map_t *out       // The struct to edit
) {
    // Open file for read and check it was successful
    FILE *meta_path_file = fopen(meta_path, "r");
    if (meta_path_file == NULL) {
        // Failed to open file
        return ERR_FILE;
    }

    char curr_line[MAX_LINE_LEN];
    char config_type[MAX_LABEL_LEN];
    char *value;

    // Check the section of each line up to the '=' character, and then
    // set the appropriate values
    while (fgets(curr_line, MAX_LINE_LEN, meta_path_file) != NULL) {
        if (sscanf(curr_line, CONFIG_LINE_FORMAT, config_type, &value) == 2) {
            if (strcmp(config_type, "NPOINTS_X") == 0) {
                out->npoints_x = atoi(value);
            } else if (strcmp(config_type, "NPOINTS_Y") == 0) {
                out->npoints_y = atoi(value);
            } else if (strcmp(config_type, "RESOLUTION") == 0) {
                out->resolution = atof(value);
            } else {
                // Invalid line detected
                return ERR_MALFORMED;
            }
        } else {
            // Invalid line detected
            return ERR_MALFORMED;
        }
    }

    // Close file and check it was successful
    if (fclose(meta_path_file) != 0) {
        // Failed to close file
        return ERR_FILE;
    }

    return ERR_OK;
}

// Sets the data of the file from the csv file
static enum io_error_t load_map_data(
    const char *data_path,                // The file to read from
    struct terrain_map_t *out       // The struct to edit
) {
    // Open file for read and check it was successful
    FILE *data_path_file = fopen(data_path, "r");
    if (data_path_file == NULL) {
        // Failed to open file
        return ERR_FILE;
    }

    // Initialise variables for fscanf later
    phys_dist_t x = 0;
    phys_dist_t y = 0;
    phys_dist_t elevation = 0;
    uint8_t cost = 0;

    phys_dist_t x_old = 0;
    phys_dist_t y_old = 0;

    int index = 0;

    // Skip header of csv file
    fscanf(data_path_file, "%*1[\n]");

    // Check the validity of each line of data and set the appropriate values
    while (
        fscanf(data_path_file, "%f,%f,%f,%c", &x, &y, &elevation, &cost) != EOF
    ) {
        // Check x and y values are valid
        if (
            x < x_old || 
            (y < y_old && y != 0) || 
            x >= out->npoints_x || 
            y >= out->npoints_y
        ) {
            return ERR_MALFORMED;
        }

        // Update values
        x_old = x;
        y_old = y;
        out->data_elevation[index] = elevation;
        out->data_cost[index] = cost;
        index++;
    }

    // Close file and check it was successful
    if (fclose(data_path_file) != 0) {
        // Failed to close file
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
    load_meta_path(meta_path, out);
    load_data_path(data_path, out);

    return ERR_OK;
}

// Store data from out into csv output file
enum io_error_t store_path(
    const char *out_path,
    const struct terrain_path_t *path
) {
    // Open file for write and check it was successful
    FILE *out_path_file = fopen(out_path, "w");
    if (out_path_file == NULL) {
        // Failed to open file
        return ERR_FILE;
    }

    // Print header to csv file
    fprintf(out_path_file, "x,y");

    // Print all x and y values in path to output file
    for (int i = 0; i < path->npoints; i++) {
        fprintf(out_path_file, "%f,%f\n", path->waypoints->x, path->waypoints->y);
    }

    // Close file and check it was successful
    if (fclose(out_path_file) != 0) {
        // Failed to close file
        return ERR_FILE;
    }

    return ERR_OK;
}
