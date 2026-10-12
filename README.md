# ESP32-H2 tank water level

Battery-powered water level sensor. An ESP32-H2 reads a VL53L0X time-of-flight sensor over I2C and reports to Zigbee as a **Bacchus Water level meter**, which stock zigbee2mqtt already knows. No external converter is required.

The chip sleeps in deep sleep between readings. It wakes every minute, measures, and turns the radio on only when the level has moved by more than 0.5 cm or 15 minutes have passed since the last report. Those two periods are `MEASURE_INTERVAL_SEC`, `REPORT_INTERVAL_SEC`, and `LEVEL_CHANGE_THRESHOLD_CM` at the top of `main/app_config.h`.

## Wiring

Defaults are in `main/app_config.h`.

| Function | GPIO | Notes |
| --- | --- | --- |
| VL53L0X SDA | 10 | 3.3 V I2C. Released and held high in deep sleep, so the module pull-ups draw nothing. |
| VL53L0X SCL | 11 | Same as SDA. |
| VL53L0X XSHUT | 12 | Held low in deep sleep. Released only while a reading is in progress. |
| Battery ADC | 1 | Middle node of the divider. |
| Divider sink | 3 | Bottom resistor returns here, not to GND. Driven low only while sampling. Set `BATTERY_SINK_GPIO` to `-1` if the resistor is tied to GND. |
| Reset button | 13 | To GND. Wakes the chip and factory-resets the Zigbee network. |

Tie the module VIN to 3.3 V. Do not power it from a GPIO. Ranging draws about 20 mA with higher peaks. XSHUT low puts the sensor in hardware standby between readings. Each shot uses a 20 ms timing budget (`VL53L0X_TIMING_BUDGET_US`). Mount the sensor above the water, looking down.

```text
3V3 ------------------- VL53L0X VIN
GPIO10 ---------------- VL53L0X SDA
GPIO11 ---------------- VL53L0X SCL
GPIO12 ---------------- VL53L0X XSHUT
GND ------------------- VL53L0X GND
```

Leave the module's interrupt pin unconnected.

A high-side load switch on VIN can remove the module regulator's idle current as well. GPIO14 is free for its enable pin. If that switch opens during deep sleep, SDA and SCL must be driven low and held there. Released-high lines back-feed an unpowered module through its I2C pull-ups. That drive-low behaviour is not what `vl53l0x_shutdown()` does now.

## Battery

A single Li-ion cell is about 4.2 V full and 3.3 V empty. Do not connect it to the board's **5V** pin or to **3V3** directly. The onboard regulator expects about 5 V, and a raw cell at 4.2 V is above the ESP32-H2 supply limit. Use a separate 3.3 V regulator. Unplug USB while the regulator is connected.

Any of these will do. The first three sleep at 1–4 µA, which suits a device that is asleep almost all of the time. Output current of 200 mA covers the Zigbee peaks. Near empty, a low-dropout regulator sags with the cell and the chip keeps running down toward 3.0 V.

| Part | Package | Idle current | Output | Notes |
| --- | --- | --- | --- | --- |
| MCP1700-3302E/TO | TO-92 | 1.6 µA | 250 mA | Easiest to solder. Preferred choice. |
| HT7333-A | TO-92 | about 2–4 µA | 250 mA | Buy the **-A** pinout. The -B part swaps pins. |
| XC6206P332MR | SOT-23 | about 1 µA | about 200 mA | Small. Same SOT-23 pin order as the MCP1700 SOT-23. |
| MCP1700T-3302E/TT | SOT-23 | 1.6 µA | 250 mA | Same chip as the first row, smaller package. |
| TPS63001 (or a 3.3 V buck-boost module built on it) | module or chip | tens of µA | 800 mA class | Holds 3.3 V across the whole cell range. Costs more sleep current than an LDO. |

TO-92 pin order for **MCP1700-3302E/TO** and **HT7333-A**, flat face toward you, pins down:

| Pin | Name |
| --- | --- |
| 1 | GND |
| 2 | VIN |
| 3 | VOUT |

SOT-23 pin order for **XC6206P332MR** and **MCP1700T-3302E/TT**, pins down, pin 1 on the left:

| Pin | Name |
| --- | --- |
| 1 | GND |
| 2 | VOUT |
| 3 | VIN |

```text
cell+ ----+---- regulator VIN
          |         VOUT ---- ESP32-H2 3V3
          |         GND  ---- ESP32-H2 GND ---- cell-
          |
          +---- 100k ----+---- GPIO1 (ADC)
                         |
                       100k
                         |
                       GPIO3 (divider sink)
```

Put 1 µF ceramic from VIN to GND and 1 µF ceramic from VOUT to GND, both next to the regulator. 10 µF on the output is fine for the HT7333-A.

GPIO1 only measures the cell. GPIO3 is driven low during that sample so the two 100 kΩ resistors do not drain the pack in sleep. `BATTERY_EMPTY_MV` (3300) and `BATTERY_FULL_MV` (4200) set the 0% and 100% points. The reported Zigbee percentage uses the standard half-percent units on the Power Configuration cluster.

A buck-boost module uses the same three wires: cell positive to its VIN, cell negative and the ESP32 GND to its GND, and its 3.3 V output to the ESP32 **3V3** pin. Keep the divider on the cell, not on the regulated 3.3 V.

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
