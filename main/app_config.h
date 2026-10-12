#pragma once

#include <stdint.h>

/*
 * Edit these three values. Everything else has a sensible default.
 *
 * The device wakes on MEASURE_INTERVAL_SEC, reads the sensor, then goes
 * back to deep sleep. The Zigbee radio stays off unless the water level
 * moved by more than LEVEL_CHANGE_THRESHOLD_CM, or REPORT_INTERVAL_SEC
 * has passed since the last successful report.
 *
 * REPORT_INTERVAL_SEC must stay below the parent timeout. The firmware
 * asks for a 256 minute end-device timeout, so keep the report interval
 * shorter than that.
 */
#define MEASURE_INTERVAL_SEC 60u
#define REPORT_INTERVAL_SEC (15u * 60u)
#define LEVEL_CHANGE_THRESHOLD_CM 0.5f

/* How long to stay awake after a normal report, so the parent can ack. */
#define AWAKE_AFTER_REPORT_SEC 8u
/* First join stays up longer so zigbee2mqtt can finish its interview. */
#define AWAKE_AFTER_JOIN_SEC 25u

#define ZIGBEE_ENDPOINT 1
/* Home Automation profile device id. zigbee2mqtt matches the model string. */
#define ZIGBEE_DEVICE_ID 0x0302
/* Channels 11-26. The coordinator can be on any of them. */
#define ZIGBEE_CHANNEL_MASK 0x07FFF800u

/*
 * zigbee2mqtt matches the Basic model identifier exactly. The length
 * prefix is added when the cluster is created.
 */
#define ZIGBEE_MANUFACTURER_TEXT "Bacchus"
#define ZIGBEE_MODEL_TEXT "Bacchus Water level meter"

/*
 * tank_height is the one Bacchus dimension: sensor to the bottom of the
 * tank, in centimeters. Water level is that height minus the VL53L0X
 * distance. Filling percent uses the same height as 100%.
 */
#define DEFAULT_TANK_HEIGHT_CM 100u
#define DEFAULT_ALARM_MAX_PERCENT 90u
#define DEFAULT_ALARM_MIN_PERCENT 10u

/*
 * Wiring, ESP32-H2-DevKitM-1 friendly. Change the GPIOs to match the board.
 *
 * VL53L0X supply is switched by VL53L0X_POWER_GPIO so the module draws
 * nothing in deep sleep. That pin must be an LP GPIO (7-14); other pins
 * lose their level when the chip sleeps. High turns the supply on.
 * Use a high-side load switch. Do not feed the module from the GPIO
 * itself: ranging draws about 20 mA, with higher peaks.
 *
 *   3V3 -- load switch -- VL53L0X VIN
 *              |
 *         VL53L0X_POWER_GPIO
 *
 * XSHUT stays low until the supply is up, then the driver releases it
 * for the reading and pulls it low again before the supply is removed.
 *
 * Battery: a divider from the pack to BATTERY_ADC_GPIO, with the bottom
 * resistor returning to BATTERY_SINK_GPIO instead of GND. The sink pin
 * is driven low only during the measurement, so the divider does not
 * drain the pack in deep sleep. Set BATTERY_SINK_GPIO to -1 if the
 * bottom resistor is tied straight to GND.
 *
 *   pack+ -- Rhigh --+-- BATTERY_ADC_GPIO
 *                    |
 *                   Rlow
 *                    |
 *              BATTERY_SINK_GPIO
 *
 * Factory reset: a button from FACTORY_RESET_GPIO to GND. It wakes the
 * chip from deep sleep. GPIO13 is a low-power pin on the H2, so it can
 * wake the chip. Do not use GPIO9; holding it at reset enters the ROM
 * download mode.
 */
#define I2C_SDA_GPIO 10
#define I2C_SCL_GPIO 11
#define VL53L0X_XSHUT_GPIO 12
/* LP GPIO. Held low through deep sleep so the load switch stays off. */
#define VL53L0X_POWER_GPIO 14
#define VL53L0X_POWER_ON_LEVEL 0 // 1 when using a TPS22918 low-side switch, and 0 when using an AO3401 high-side switch
#define VL53L0X_I2C_ADDR 0x29
/*
 * Shortest ranging budget the VL53L0X allows with its default steps.
 * Raise this if the water surface is missed; each extra millisecond is
 * time the laser is on.
 */
#define VL53L0X_TIMING_BUDGET_US 20000

#define BATTERY_ADC_GPIO 1
#define BATTERY_SINK_GPIO 3
#define BATTERY_DIVIDER_HIGH 100000
#define BATTERY_DIVIDER_LOW 100000
#define BATTERY_EMPTY_MV 3300
#define BATTERY_FULL_MV 4200

#define FACTORY_RESET_GPIO 13

/* VL53L0X readings at or above this are "no target". */
#define VL53L0X_INVALID_MM 2500
