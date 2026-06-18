#include "smooth.h"

#include <assert.h>

struct point_2d_t lerp(const struct point_2d_t a, const struct point_2d_t b, float c) {
  if (c < 0) c = 0;
  if (c > 1) c = 1;
  return (struct point_2d_t){
    a.x * c + b.x * (1-c),
    a.y * c + b.y * (1-c)
  };
}

uint8_t cost(const struct terrain_map_t *map, const struct point_2d_t point) {
  const uint32_t index = terrain_map_get_pos(map,point);
  assert(index < map->npoints_x * map->npoints_y);
  return map->data_cost[index];
}

// Single smooth step
void step(const struct terrain_map_t *map, const struct dynarr_t *path) {
  if (path->length < 2) return;
  struct point_2d_t first = *(struct point_2d_t*)dynarr_get(path,0);
  struct point_2d_t* second = dynarr_get(path,1);
  for (size_t i = 1; i < path->length; i++) {
    struct point_2d_t* third = dynarr_get(path,i);
    const struct point_2d_t base = lerp(first,*third,0.5f); // Midpoint of first and third
    const struct point_2d_t new = lerp(*second,base,0.5f - (float)(cost(map,*second) - cost(map,base))/10.0f);
    first = *second; // Second moves to first
    *second = new; // The actual path changes
    second = third; // Third moves to second
  }
}

// Smooth entire path using multiple steps
void smooth(
  const struct terrain_map_t *map,
  const struct dynarr_t *path
) {
  for (int i = 0; i < SMOOTH_STEPS; i++) {
    step(map,path);
  }
}
