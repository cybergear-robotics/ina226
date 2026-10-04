# INA226 component for ESP-IDF

[![Examples build](https://github.com/cybergear-robotics/ina226/actions/workflows/build_example.yml/badge.svg)](https://github.com/cybergear-robotics/ina226/actions/workflows/build_example.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/cybergear-robotics/ina226/graphs/commit-activity)
[![Framework](https://img.shields.io/badge/Framework-ESP_IDF-orange.svg)](https://shields.io/)
[![Language](https://img.shields.io/badge/Language-C-purple.svg)](https://shields.io/)

## Using component

```bash
idf.py add-dependency "cybergear-robotics/ina226"
```

## Requirements

ESP-IDF 5.2 or newer is required. Configure an I2C master bus and add the
INA226 as an I2C device using `driver/i2c_master.h`, then pass the resulting
device handle through `ina226_config_t.i2c_dev`. The example below shows the
complete setup.

## Example

Not all examples are ported. For further examples, please look at the original project.

1. create example project

```bash
idf.py create-project-from-example "cybergear-robotics/ina226:read_voltage"
```

2. Go to to example directory (for example `read_voltage`)
   `cd read_voltage`
3. Set ESP chip
   `idf.py set-target esp32`
4. Configure I2C settings
   `idf.py menuconfig`
5. Build, flash
   `idf.py build flash monitor`

## Hardware test

The Unity hardware test is self-contained in `test/` and uses I2C port 0, SCL GPIO 11,
and SDA GPIO 12 by default. It expects 24.0 V on the INA226 bus input, with a 0.5 V
tolerance. All values can be changed with `idf.py menuconfig`.

Run it from the component repository:

```bash
idf.py -C test_app build flash monitor
```
