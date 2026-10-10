# ESP-NOW Sensor PlatformIO Project Plan

## Goal and scope

Build a standalone PlatformIO firmware project for the repository's ESP-12F/BME280 sensor board. It samples the BME280, updates the integrated OLED, and replies to requests from an ESP-NOW Store. This plan replaces the incomplete Arduino sketch; it is not buildable firmware.

## Hardware and display

- Target the ESP8266 Arduino framework. Confirm the module's flash size and choose the matching PlatformIO board ID before creating `platformio.ini`.
- BME280: I2C address `0x76`; OLED: SSD1306 128x32 at `0x3c`; shared I2C uses SDA GPIO2 and SCL GPIO0, as in the board firmware and ESPHome YAML.
- Mirror the ESPHome display content: temperature in °C and relative humidity in %, both to one decimal place, with `°C temp` and `% hum` labels. Show `WARN!` at or above the configured humidity threshold; default to 65.0% to match `ESPHome-HumiTempSensor.yaml`. Support 0°/180° orientation.
- Pressure is measured and sent to the Store but is not shown on the ESPHome display. ESPHome also dims by sun elevation; the standalone sensor has no time/location source, so treat matching solar dimming as a follow-up requiring Store-provided brightness or a local time/location source. Define and clamp the contrast range before implementation.

## Firmware responsibilities

Initialize the sensor, display, serial diagnostics, and ESP-NOW without joining an access point. Use a configured Store MAC and a shared Wi-Fi channel; handle channel selection as part of pairing. Sample on a 10-second cadence to match the ESPHome YAML, refresh the display, and serve the latest reading on a Store request. Return sample age or a status when no valid reading is available. Keep ESP-NOW callbacks short: validate and copy packets into a queue, then perform I2C, display, and sensor work in the main loop.

## Protocol plan

The old sketch proposed command `0x00` for config, `0x01` for a reading request, and `0x02` for pressure/temperature/humidity data. Treat these as draft message types, not a stable wire format. Define a versioned packet header and fixed-width fields before coding; avoid transmitting raw compiler-dependent `float` structs. Include message type, sequence number, and status. Config should carry threshold and orientation; validate lengths, version, ranges, sender MAC, and duplicate sequence numbers. Add pairing/acknowledgement behavior so the Store can register a sensor and deliver config without persisting Store settings on the sensor. Review the repository's previously referenced WifiEspNow library and select a pinned ESP-NOW API/abstraction that can be used consistently by both PlatformIO projects; verify callback and peer APIs for both chip families before implementation.

## Implementation sequence and acceptance

1. Create `platformio.ini`, `src/main.cpp`, and a dependency list for the ESP8266 Arduino core, BME280, and SSD1306 libraries.
2. Bring up I2C, sample and format display values, including warning boundary and screen orientation.
3. Add the versioned ESP-NOW protocol, pairing/configuration, request/reply flow, timeout handling, and serial diagnostics.
4. Verify compilation for the selected board; on hardware verify readings, OLED layout, warning threshold, orientation, Store pairing, and reliable replies at the configured interval.

Keep credentials out of firmware and repository files. Record the selected board ID, library versions, channel strategy, and any hardware-tested limitations in the project README when implementation begins.
