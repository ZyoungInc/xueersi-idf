#ifndef MOTION_LEVEL_H
#define MOTION_LEVEL_H

#include <stdbool.h>
#include <stdint.h>

#include "lvgl.h"

lv_obj_t *motion_level_create(lv_obj_t *parent, uint32_t theme_color, uint32_t ink_color);
bool motion_level_calibrate(lv_obj_t *obj);
void motion_level_update(lv_obj_t *obj, const int16_t acc[3], bool present);

#endif
