# ESP32-H2 tank water level

Battery-powered water level sensor. An ESP32-H2 reads a VL53L0X time-of-flight sensor over I2C and reports to Zigbee as a Tuya **872WZ** (`TS0601`, manufacturer `_TZE204_7yyuo8sr`), which zigbee2mqtt already knows.

The chip sleeps in deep sleep between readings. It wakes every minute, measures, and turns the radio on only when the level has moved by more than 0.5 cm or 15 minutes have passed since the last report. Those two periods are `MEASURE_INTERVAL_SEC`, `REPORT_INTERVAL_SEC`, and `LEVEL_CHANGE_THRESHOLD_CM` at the top of `main/app_config.h`.

## Wiring

Defaults are in `main/app_config.h`.

| Function | GPIO | Notes |
| --- | --- | --- |
| VL53L0X SDA | 10 | 3.3 V I2C. Most modules already have pull-ups. |
| VL53L0X SCL | 11 | |
| VL53L0X XSHUT | 12 | Held low during deep sleep so the sensor stays in standby. |
| Battery ADC | 1 | Middle node of the divider. |
| Divider sink | 3 | Bottom resistor returns here, not to GND. Driven low only while sampling. Set `BATTERY_SINK_GPIO` to `-1` if the resistor is tied to GND. |
| Reset button | 13 | To GND. Wakes the chip and factory-resets the Zigbee network. |

VL53L0X VIN and GND go to 3.3 V and ground. Mount the sensor above the water, looking down.

Battery divider, for a single Li-ion cell and the default 1:1 resistors (100 kΩ / 100 kΩ):

```text
pack+ ---- Rhigh ----+---- GPIO1 (ADC)
                     |
                   Rlow
                     |
                   GPIO3 (sunk to GND only during the reading)
```

`BATTERY_EMPTY_MV` (3300) and `BATTERY_FULL_MV` (4200) set the 0% and 100% points. The reported Zigbee percentage uses the standard half-percent units on the Power Configuration cluster.

## Tank size

Until you set them from zigbee2mqtt, installation height and maximum depth are both 1.00 m.

* **installation_height** is the distance from the sensor to the bottom of the tank.
* **liquid_depth_max** is the water depth that should read as 100%.
* **min_set** / **max_set** are the percent thresholds for `liquid_state` `low` and `high`.

`liquid_depth` is `installation_height` minus the distance the VL53L0X measures. zigbee2mqtt shows it in meters. The device sends centimeters, which is what the current 872WZ converter expects (`value / 100 = meters`).

`silent_mode` and `alarm_duration` are stored and reported so the 872WZ controls round-trip, but this board has no buzzer.

## Build (ESP-IDF on Windows)

Use ESP-IDF 5.3 or newer and esp-zigbee-lib 2.x (the project pulls `^2.0.4`). From an ESP-IDF PowerShell prompt:

```bat
cd C:\Arduino\ESP32-H2-waterlevel\esp32-h2-waterlevel
idf.py set-target esp32h2
idf.py build
idf.py -p COM5 flash monitor
```

Replace `COM5` with the board's port. The monitor shows a boot every minute. When the level is stable you will see `radio off` and the Zigbee stack will not start.

## Pairing

1. In zigbee2mqtt, permit joining.
2. Power the board. The first boot (and any later boot that has not joined) turns the radio on and steers.
3. The device interviews as **Tuya 872WZ**.

`liquid_state`, `liquid_level_percent`, and `liquid_depth` publish with the stock zigbee2mqtt definition. No custom converter is required for the water level.

The real 872WZ is defined as a mains-powered device, so stock zigbee2mqtt does not publish its battery. This firmware still reports battery voltage and percentage on the standard Power Configuration cluster. To see them in MQTT, copy `z2m/872WZ-battery.js` to `data/external_converters/` and restart zigbee2mqtt. That file is the 872WZ profile plus battery.

Hold the GPIO13 button to wake the chip and erase the Zigbee network, then permit join again. Do not use the BOOT button (GPIO9) for this; holding it during reset enters the ROM download mode.

## Power notes

Deep sleep reboots the chip. After each report it has to rejoin its parent, which costs more energy than a light-sleep poll. That tradeoff is why the radio stays off on the minutes where the level has not changed. The parent is asked for a 256 minute end-device timeout, so keep `REPORT_INTERVAL_SEC` below that or the coordinator will drop the device.
