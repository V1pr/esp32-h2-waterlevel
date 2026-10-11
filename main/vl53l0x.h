#pragma once

#include "esp_err.h"
#include <stdint.h>

/* Release XSHUT and run the sensor init. */
esp_err_t vl53l0x_begin(void);

/* One single-shot range, in millimeters. Call vl53l0x_begin() first. */
esp_err_t vl53l0x_read_once(uint16_t *distance_mm);

/* Drive XSHUT low and hold it through deep sleep. Safe to call more than once. */
void vl53l0x_shutdown(void);
