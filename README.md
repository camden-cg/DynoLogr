# DynoLogr

DynoLogr is an ESP32-based vehicle telemetry project with an OLED interface and
a browser dashboard.

## Repository layout

- `firmware/DynoLogr/` contains the Arduino sketch. `DynoLogr.ino` holds shared
  configuration and state; the other `.ino` tabs group control, IMU, display,
  web API, and application code.
- `firmware/prototypes/` contains the standalone OLED/MPU prototype sketch.
- `software/` contains the browser dashboard and its CSS and JavaScript.
- `electronics/` contains the KiCad board file.
- `docs/images/` contains project images.

## Firmware

Open `firmware/DynoLogr/DynoLogr.ino` in Arduino IDE. This sketch uses
Bluepad32, which requires its ESP32 board package; installing a `Bluepad32`
library in the sketchbook `libraries` folder is not sufficient.

In Arduino IDE settings, add both board package URLs:

- `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
- `https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json`

In Boards Manager, install `esp32` and `esp32_bluepad32`. Then select **DOIT
ESP32 DEVKIT V1** from the **ESP32 + Bluepad32 Arduino** board menu, rather
than selecting the same board from the regular Espressif board menu. See the
[Bluepad32 Arduino setup guide](https://bluepad32.readthedocs.io/en/latest/plat_arduino/).

Install the remaining libraries included by the sketch: Adafruit GFX,
Adafruit SSD1306, Adafruit MPU6050, Adafruit Unified Sensor, ESP32Servo, and
ArduinoJson. The ESP32 Arduino core provides Wire, WiFi, and WebServer.

Copy `firmware/DynoLogr/Credentials.example.h` to
`firmware/DynoLogr/Credentials.h` and set `WIFI_SSID` and `WIFI_PASS` there.
`Credentials.h` is ignored by Git and should remain local. The firmware continues
to run without Wi-Fi.

## Dashboard

Open `software/index.html` in a browser. Connect to the ESP32's network address
using the dashboard connection controls.
