#pragma once

#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool distance_valid;
    int32_t distance_mm;
    int32_t depth_mm;
    uint8_t percent;
    uint8_t liquid_state;
    uint8_t battery_percent;
    uint16_t battery_mv;
} water_sample_t;

typedef void (*water_activity_cb_t)(void);

void water_settings_init(void);
bool water_network_joined(void);
void water_network_set_joined(bool joined);

/* Called when a Zigbee write arrives, so the app can stay awake a bit longer. */
void water_set_activity_callback(water_activity_cb_t cb);

void water_set_reading(bool distance_valid, int32_t distance_mm, uint8_t battery_percent, uint16_t battery_mv);
const water_sample_t *water_sample(void);

/* Register the Basic, Power Configuration, and Tuya 0xEF00 clusters. */
esp_err_t water_zigbee_device_create(void);

/*
 * Send the 872WZ datapoints to the coordinator.
 * Call from the Zigbee task, or from another task while holding the Zigbee lock.
 * Returns true when the depth report was accepted by the stack.
 */
bool water_send_tuya_report(void);

/* Report BatteryPercentageRemaining on the standard Power Configuration cluster. */
void water_send_battery_report(void);
