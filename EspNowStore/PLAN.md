# ESP-NOW Store PlatformIO Project Plan

## Goal and scope

Build a PlatformIO firmware project for an ESP32-WROOM-32 board that coordinates ESP-NOW sensor nodes, stores their readings on SD, and provides local status through buttons and a WS2812B LED string. This plan replaces the incomplete Arduino sketch; it is not buildable firmware.

## Hardware map

| Function | GPIO / connection | Notes |
| --- | --- | --- |
| BUT_CONF | GPIO34, active low | GPIO34 is input-only and has no internal pull-up; provide an external pull-up. |
| BUT_USR | GPIO13, active low | Configure pull-up if the board wiring permits. Its user action remains to be defined. |
| WS2812B data | GPIO17 | Drives 9 LEDs. |
| LED power MOSFET | GPIO16, active high | Keep low during boot; enable before sending LED data and allow power to settle. |
| SD CS | GPIO4 | Chip select. |
| SD DI | GPIO5 | SPI MOSI (controller-to-card). |
| SD CL | GPIO12 | SPI SCK. |
| SD DO | GPIO15 | SPI MISO (card-to-controller). |
| SD power | Board supply | Use the slot/card's required supply and common ground. |

GPIO12 and GPIO15 are ESP32 strapping pins. Before implementation, verify that the SD slot and card do not drive incompatible levels during reset; explicitly initialize SPI with SCK=12, MISO=15, MOSI=5, CS=4. Confirm the WS2812B supply voltage, common ground, and whether a 3.3V-to-LED data level shifter is required.

## Firmware responsibilities

Use the ESP32 Arduino framework and ESP-NOW in station mode. Maintain a configured list of known sensor MAC addresses, pair new nodes through BUT_CONF, send each node its threshold/orientation settings, and periodically request measurements. ESP-NOW callbacks should validate and queue packets only; perform SD writes, LED updates, and scheduling from the main loop or dedicated tasks. Track last-seen time and retry/backoff state for unavailable nodes rather than dropping them on one send failure.

Initialize the LED power switch off, then enable it before driving the 9-pixel string. Define documented LED patterns for startup, pairing, card errors, and sensor health. Keep BUT_CONF for configuration/pairing. Decide and document BUT_USR behavior before implementing its event handler.

## Storage, time, and radio

Use the SD card for versioned configuration (known sensors, display threshold/orientation, optional Wi-Fi/time settings) and append-only measurement records. Define record fields for timestamp or an explicit “time unavailable” marker, sensor MAC, sequence number, temperature, humidity, pressure, and status. Flush safely and handle missing/full/read-only cards without blocking radio callbacks. The original sketch proposed Wi-Fi/NTP; make Internet time optional and keep ESP-NOW operating without it. Wi-Fi association and ESP-NOW must share a channel, so choose and document a channel policy before pairing nodes.

## Protocol and implementation sequence

Use the Sensor plan's versioned fixed-width protocol: pairing/ack, config, reading request, and measurement/status replies. Validate packet version, type, length, sender, sequence, and value ranges. Do not use the old raw-float packet sketch as a stable wire format.

1. Create `platformio.ini` for an ESP32-WROOM-32 compatible board, plus `src/main.cpp` and dependencies.
2. Bring up buttons, MOSFET-controlled LEDs, custom-pin SPI, and SD configuration/logging.
3. Add ESP-NOW discovery/pairing, config delivery, polling, retries, time synchronization, and status patterns.
4. Verify compilation and test boot strapping with the SD card installed, button behavior, LED power/data sequencing, card failure handling, multiple sensors, and radio recovery.

Keep Wi-Fi credentials off-card only if the storage medium's physical access is a concern; never commit real credentials. Document the selected board definition, SPI frequency, SD format, radio channel policy, LED pattern meanings, and BUT_USR behavior when implementation begins.
