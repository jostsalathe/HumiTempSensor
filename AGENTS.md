# Repository Guidelines

## Project Structure & Module Organization

The root contains the main legacy ThingsBoard Arduino sketch (`HumiTempSensor.ino`), the supported ESPHome configuration (`ESPHome-HumiTempSensor.yaml`), and the README. `EspNowSensor/PLAN.md` and `EspNowStore/PLAN.md` describe planned future PlatformIO projects; no ESP-NOW firmware is currently implemented. The `ESP-12F_BME280_RevD_*.pdf` files document the board layout and schematic. There is no separate source, test, or asset tree.

## Build, Test, and Development

- `esphome compile ESPHome-HumiTempSensor.yaml` compiles the ESPHome firmware. Configure Wi-Fi secrets and replace the example API/OTA values first.
- The legacy Arduino sketch can be opened in Arduino IDE with the matching ESP8266 board and libraries listed in `README.md`. The ESP-NOW directories are plans only; their future PlatformIO projects do not yet have build configurations.
- No automated test suite is present. For hardware changes, flash the firmware and check sensor readings, display output, LEDs, and Wi-Fi or ESP-NOW behavior as applicable.

## Coding Style & Naming

Keep Arduino code in the existing `.ino` file and follow its two-space indentation, brace placement, and descriptive camelCase function/variable names. Use `UPPER_SNAKE_CASE` for pin and compile-time constants. Keep ESPHome substitutions and secrets near the top of the YAML configuration; do not commit real credentials or device keys.

## Testing Guidelines

There are no test files or coverage requirements. Compile the ESPHome YAML after edits to it. Compile Arduino changes with their required board packages and libraries, then check on target hardware; once the planned PlatformIO projects are implemented, apply the same compile-and-hardware check to them. Note any untested hardware behavior in the change description.

## Commit & Pull Request Guidelines

History uses short descriptive commit subjects and sometimes prefixes work-in-progress commits with `WIP:`. Use a concise subject that names the change. Pull requests should summarize the hardware or firmware impact, list the build or device checks performed, and include display photos or wiring details when they clarify a hardware-facing change. Remove credentials and device tokens from examples and screenshots.

## Configuration & Architecture Notes

ESPHome is the primary documented setup. The root Arduino sketch is a legacy ThingsBoard implementation, while the ESP-NOW projects are documented plans with no firmware yet. Preserve those distinctions in documentation and avoid presenting planned flows as supported features. Keep board pin assignments consistent with the schematics and existing firmware.
