# ESP32-H2 tank water level

Battery-powered water level sensor. An ESP32-H2 reads a VL53L0X time-of-flight sensor over I2C and reports to Zigbee as a **Bacchus Water level meter**, which stock zigbee2mqtt already knows. No external converter is required.

The chip sleeps in deep sleep between readings. It wakes every minute, measures, and turns the radio on only when the level has moved by more than 0.5 cm or 15 minutes have passed since the last report. Those two periods are `MEASURE_INTERVAL_SEC`, `REPORT_INTERVAL_SEC`, and `LEVEL_CHANGE_THRESHOLD_CM` at the top of `main/app_config.h`.

## Wiring

Defaults are in `main/app_config.h`.

| Function | GPIO | Notes |
| --- | --- | --- |
| VL53L0X SDA | 10 | 3.3 V I2C. Driven low during deep sleep so it cannot feed the unpowered module. |
| VL53L0X SCL | 11 | Same as SDA. |
| VL53L0X XSHUT | 12 | Released only while a reading is in progress. |
| VL53L0X power | 14 | High enables a high-side load switch for the module VIN. Held low in deep sleep. |
| Battery ADC | 1 | Middle node of the divider. |
| Divider sink | 3 | Bottom resistor returns here, not to GND. Driven low only while sampling. Set `BATTERY_SINK_GPIO` to `-1` if the resistor is tied to GND. |
| Reset button | 13 | To GND. Wakes the chip and factory-resets the Zigbee network. |

Do not tie VL53L0X VIN straight to 3.3 V, and do not power the module from a GPIO. Ranging draws about 20 mA with higher peaks. GPIO14 only enables a high-side load switch, and only for the reading. Each shot uses a 20 ms timing budget (`VL53L0X_TIMING_BUDGET_US`). Mount the sensor above the water, looking down.

Use a **TPS22918** in SOT-23-6 (orderable part **TPS22918DBVR**). ON is active high, which matches the firmware. Off current is about 0.5 µA. Pin 1 is the pin by the package mark, looking at the top of the chip:

```text
        mark
     +--------+
VIN  |1      6| VOUT
GND  |2      5| QOD
ON   |3      4| CT
     +--------+
```

| Pin | Name | Connect to |
| --- | --- | --- |
| 1 | VIN | ESP32-H2 3V3, with 1 µF ceramic to GND at the pin |
| 2 | GND | GND |
| 3 | ON | GPIO14, and a 100 kΩ pulldown to GND |
| 4 | CT | Leave open. A capacitor here only slows turn-on. |
| 5 | QOD | VOUT (pin 6), so the sensor supply is discharged when the switch opens |
| 6 | VOUT | VL53L0X VIN |

ON must not float. The pulldown keeps the switch off until the firmware drives GPIO14. The firmware holds that pin low through deep sleep.

```text
3V3 ----+---- TPS22918 VIN (1)
        |         VOUT (6) ---- VL53L0X VIN
       1µF        QOD (5) ---- VOUT
        |         CT (4) open
       GND        ON (3) ----+---- GPIO14
                             |
                           100k
                             |
                            GND

GPIO10 ---------------- VL53L0X SDA
GPIO11 ---------------- VL53L0X SCL
GPIO12 ---------------- VL53L0X XSHUT
GND ------------------- VL53L0X GND
```

Leave the module's interrupt pin unconnected.

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

Until you set it from zigbee2mqtt, tank height is 100 cm.

**tank_height** is the distance from the sensor to the bottom of the tank. `water_level` is that height minus the distance the VL53L0X measures, shown in centimeters (0.1 cm steps). `filling` is `water_level / tank_height`, in percent.

`alarm_min_threshold` and `alarm_max_threshold` are stored so the Bacchus controls round-trip. This board has no alarm output. `invert_threshold` is stored the same way.

`measurment_period` (zigbee2mqtt spells it that way) is stored and defaults to 15 minutes. It does not change the sleep schedule. Wake and transmit times stay `MEASURE_INTERVAL_SEC`, `REPORT_INTERVAL_SEC`, and `LEVEL_CHANGE_THRESHOLD_CM` in `main/app_config.h`.

`out_of_service` is true when the VL53L0X has no valid target. The Bacchus profile also exposes `temperature`; this board has no temperature sensor, so that entity stays at 0 °C. `battery` and `voltage` come from the standard Power Configuration cluster.

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
3. The device interviews as **Bacchus Water level meter**.

`water_level`, `filling`, `battery`, and `voltage` publish from the stock definition. If this board was previously paired as a Tuya 872WZ, remove that device in zigbee2mqtt and pair it again. The model identifier has changed.

Hold the GPIO13 button to wake the chip and erase the Zigbee network, then permit join again. Do not use the BOOT button (GPIO9) for this; holding it during reset enters the ROM download mode.

## Power notes

Deep sleep reboots the chip. After each report it has to rejoin its parent, which costs more energy than a light-sleep poll. That tradeoff is why the radio stays off on the minutes where the level has not changed. The parent is asked for a 256 minute end-device timeout, so keep `REPORT_INTERVAL_SEC` below that or the coordinator will drop the device.
