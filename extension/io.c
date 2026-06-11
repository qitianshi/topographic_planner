#include <stdint.h>
#include <stdio.h>

#include "io.h"
#include "map.h"

#define MAX_LINE_LEN 64

// Sets the metadata of the struct from the config file
static enum io_error_t load_map_meta(
    char *meta_path,                // The file to read from
    struct terrain_map_t *out       // The struct to edit
) {
    // Open file and check it was successful
    FILE *meta_path_file = fopen(meta_path, "r");
    if (meta_path_file == NULL) {
        // Faiedl to open file
        return ERR_FILE;
    }

    // Initialise variables for sscanf later
    uint32_t npoints_x = 0;
    uint32_t npoints_y = 0;
    phys_dist_t resolution = 0;

    char curr_line[MAX_LINE_LEN];
    char *config_type;
    char *value;

    // Check the section of each line up to the '=' character, and then
    // set the appropriate values
    while (fgets(curr_line, MAX_LINE_LEN, meta_path_file) != NULL) {
        if (sscanf(curr_line, "%[^=]=%s", &config_type, &value) == 2) {
            if (strcmp(value, "NPOINTS_X") == 0) {
                npoints_x = atoi(value);
            } else if (strcmp(value, "NPOINTS_Y") == 0) {
                npoints_y = atoi(value);
            } else if (strcmp(value, "RESOLUTION") == 0) {
                resolution = atof(value);
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
    char *data_path,                // The file to read from
    struct terrain_map_t *out       // The struct to edit
) {
    // Open file and check it was successful
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

    // Check the validity of each line of data and set the appropriate values
    while (
        fscanf(data_path_file, "%d,%d,%f,%d", &x, &y, &elevation, &cost) != EOF
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
