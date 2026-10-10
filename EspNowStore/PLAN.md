# ESP-NOW Store PlatformIO Project Plan

## Goal and project shape

Implement a PlatformIO project for an ESP32-WROOM-32 that serves as the single Store for N directly connected ESP-NOW sensors. It owns operational sensor configuration, receives and logs measurements to SD, obtains wall-clock time from NTP when Wi-Fi is available, and serves sensor history over Wi-Fi. Suggested layout: `platformio.ini`, `src/main.cpp`, modules under `src/` for Wi-Fi/time, SD/config, ESP-NOW, web UI, buttons, and LEDs, plus a small `data/` folder for static web assets if they are not embedded in firmware. This plan replaces the incomplete Arduino sketch; it is not buildable firmware.

## Hardware map and bring-up risks

| Function | GPIO / connection | Notes |
| --- | --- | --- |
| BUT_CONF | GPIO34, active low | Input-only; provide an external pull-up. |
| BUT_USR | GPIO13, active low | Use a pull-up if the board wiring permits. Proposed action: request an immediate poll of all sensors. |
| WS2812B data | GPIO17 | Drives 9 LEDs. |
| LED power MOSFET | GPIO16, active high | Hold low during boot; enable before LED data and allow power to settle. |
| SD CS | GPIO4 | Chip select. |
| SD DI | GPIO5 | SPI MOSI (controller to card). |
| SD CL | GPIO12 | SPI SCK. |
| SD DO | GPIO15 | SPI MISO (card to controller). |
| SD power | Board supply | Confirm card voltage and common ground. |

GPIO12 and GPIO15 are ESP32 boot-strapping pins. Verify the SD slot/card does not force incompatible levels during reset; test cold boot with a card inserted. Initialize SPI explicitly as SCK=12, MISO=15, MOSI=5, CS=4. Confirm WS2812B voltage, common ground, and whether GPIO17 needs a 3.3V-to-LED level shifter. Use the ESP32 Arduino framework and pin PlatformIO board/core versions.

## SD configuration and sensor enrollment

Treat SD configuration as the source of truth. Use a versioned file such as `/config.json`, validate it at boot, and update by writing a temporary file, flushing it, then renaming it while retaining a last-known-good backup. Example fields:

```json
{
  "schema_version": 1,
  "wifi": [{"ssid": "home", "password": "...", "priority": 10}],
  "time": {"timezone": "Etc/UTC", "ntp_servers": ["pool.ntp.org"]},
  "radio": {"channel_policy": "connected-ap", "fallback_channel": 1},
  "sensors": [{"mac": "aa:bb:cc:dd:ee:ff", "name": "Bedroom", "key": "...", "interval_sec": 10, "warn_humidity_pct": 65.0, "rotation_deg": 0}]
}
```

Support an ordered list of Wi-Fi networks so the Store can recover after a location change: scan/try configured networks by priority, apply bounded connection timeouts and backoff, and reconnect without blocking ESP-NOW or web servicing. Keep passwords and per-sensor keys on the SD card; never return them from the web API. A new sensor should be added to the allowlist through a deliberate pairing window started by holding BUT_CONF, then named/configured in the SD file or protected setup UI. Require a unique per-sensor bootstrap key shared with that sensor's firmware. Do not silently accept every MAC heard during normal operation. If the SD config is missing/invalid, enter a visible setup/recovery mode and do not provision unknown sensors.

## Radio, channel, and supported N

Use the same versioned fixed-width protocol as the Sensor plan, with packet size no greater than 250 bytes for ESP8266 compatibility. Provide pairing/config request, full config snapshot/revision, acknowledgement/status, reading request, and measurement reply messages. Authenticate/encrypt unicast peers; use bounded retries, sequence IDs, duplicate suppression, and a queue from radio callbacks to application tasks. Poll each known sensor on its configured interval, track last-seen time and missed requests, and apply backoff without deleting a sensor after one failure. Do not write SD files or serve HTTP inside ESP-NOW callbacks.

One ESP32-WROOM-32 has one 2.4 GHz radio: ESP-NOW peers must use the Store's current Wi-Fi channel when it is joined to an AP. Different backup/location networks may use different channels, so nodes can go offline during a switch until their channel-scan/rejoin logic finds the Store again. Recommend either (a) keep all intended APs on one fixed 2.4 GHz channel, or (b) keep the current multi-network requirement and implement/test sensor channel recovery. If uninterrupted service across arbitrary channels or a large N is required, use a second radio/Store or move sensor traffic to ordinary Wi-Fi/MQTT. Define `MAX_SENSORS` before coding: ESP-IDF v6.0 documents 20 total peers and up to 17 encrypted peers, with 7 encrypted by default; older SDK versions and PlatformIO Arduino builds may differ. Check the pinned core's actual limits; if needed raise its configured encrypted-peer cap or split sensors across Stores.

## Time, logs, and web history

After joining Wi-Fi, synchronize with configured NTP servers and store all log timestamps in UTC; render local time in the browser using the configured timezone. After a successful sync, keep using the free-running system clock through temporary Wi-Fi outages and record the last sync time. After a power cycle with no RTC or NTP, mark wall time invalid until resynchronized and record boot ID plus monotonic uptime—never invent timestamps. If accurate wall-clock history across long outages or power loss is essential, add a battery-backed RTC such as DS3231; otherwise time estimates after a later sync must be marked uncertain.

Append records to daily files such as `/logs/YYYY/MM/DD.csv` with Store receive time/time-valid flag, Store boot ID/uptime, sensor MAC and name, sensor boot ID/uptime, sequence, temperature, humidity, pressure, and status. For buffered batches, include the sensor's current uptime and each reading's sample uptime; when NTP is valid and the boot ID matches, estimate event time as receive time minus the sensor uptime delta and mark it as estimated. Stream files in bounded chunks, rotate by UTC date, recover from a partial final line, and handle missing/full/read-only cards without blocking radio callbacks. Use a bounded RAM queue while SD is unavailable and show a clear error; document that a power loss can discard queued records. Set a retention/capacity policy and warn before the card fills.

Serve a mobile-friendly, read-only v1 web UI on the Store's Wi-Fi LAN (use mDNS such as `humitemp-store.local`, and print the fallback IP over serial). Show each sensor's latest values, last-seen/health state, and history chart; allow sensor/date-range filters and CSV download. Paginate or stream history from SD instead of loading all logs into RAM. Bundle UI assets or serve them from flash so the interface works without a CDN or Internet access. Protect the UI from WAN exposure; do not configure router port forwarding. A BUT_CONF long press can start a temporary setup SoftAP when no saved Wi-Fi is reachable. The SoftAP shares the radio/channel with ESP-NOW, and has no NTP until upstream Wi-Fi is available.

## Buttons, LEDs, and failure behavior

Proposed usability: BUT_CONF long press opens a time-limited pairing/setup window; BUT_USR short press requests an immediate poll of all nodes. Define LED patterns for boot, Wi-Fi/NTP state, pairing, SD failure/full, and missing sensors; use a dim low-power idle pattern. Initialize GPIO16 low, enable LED power only when required, then send data. SD failure must not stop measurements from being received; Wi-Fi failure must not stop local ESP-NOW logging; NTP failure must not stop either.

## Implementation milestones and acceptance

1. Create PlatformIO project, hardware drivers, SD mount/config validation, button debounce, and safe LED power sequencing.
2. Add multi-network Wi-Fi recovery, NTP state, channel policy, and encrypted/versioned ESP-NOW pairing with the Sensor project.
3. Add scheduled multi-sensor polling, retries, queueing, UTC/unsynced CSV logging, and card recovery.
4. Add LAN web history/API, date/sensor filters, CSV export, mDNS, and fallback setup AP.
5. Verify compile and hardware behavior with SD inserted/removed/full, every intended Wi-Fi network and channel, N sensors up to the declared cap, Store/AP reconnection, NTP loss/recovery, browser history over long files, LED and button actions, and boot strapping with the SD card installed.

Keep real Wi-Fi passwords and keys out of source control. An SD card holding credentials is readable if physically removed; if that matters, encrypt the config with a key held off-card. Record the chosen N cap, peer-encryption setting, channel recovery policy, card format, log schema, timezone behavior, and BUT_USR action when implementation begins.

## Primary references

- [ESP-NOW programming guide](https://docs.espressif.com/projects/esp-idf/en/v6.0/esp32/api-reference/network/esp_now.html)
