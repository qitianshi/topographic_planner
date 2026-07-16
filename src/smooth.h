#ifndef SMOOTH_H
#define SMOOTH_H

#include "adt/dynarr.h"
#include "map.h"

#include <stdbool.h>

#define SMOOTH_STEPS_DEFAULT 10

void smooth_step(const struct terrain_map_t* map, const struct dynarr_t* path);

void smooth(const struct terrain_map_t *map, uint32_t steps, const struct dynarr_t *path);

#endif // SMOOTH_H
