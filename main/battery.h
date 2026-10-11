#pragma once

#include "esp_err.h"
#include <stdint.h>

/* Pack voltage in millivolts and a 0-100 percent estimate. */
esp_err_t battery_read(uint16_t *millivolts, uint8_t *percent);
