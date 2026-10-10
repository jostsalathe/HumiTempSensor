# ESP-NOW Sensor PlatformIO Project Plan

## Goal and project shape

Implement a PlatformIO firmware project for the repository's ESP-12F/BME280 board. Each sensor reports to one ESP32 Store; the Store may manage many sensors. The sensor samples and displays locally, while all changeable operating settings come from the Store's SD configuration. Suggested layout: `platformio.ini`, `src/main.cpp`, and small modules under `src/` for display, BME280, radio protocol, and measurement scheduling. This plan replaces the incomplete Arduino sketch; it is not buildable firmware.

Target the ESP8266 Arduino framework. Confirm the module flash size and select the matching PlatformIO board ID before pinning dependencies. Use a single ESP-NOW API that works with the chosen ESP8266 and ESP32 Arduino cores; the repository's earlier WifiEspNow reference is a candidate to evaluate, not an assumed dependency.

## Hardware and display behavior

- BME280: I2C `0x76`; SSD1306 OLED: `0x3c`, 128x32; shared bus SDA GPIO2 and SCL GPIO0.
- Preserve the existing board's status LEDs on GPIO4/5 and active-low debug strap on GPIO14 if used. GPIO0 is both the I2C clock and ESP8266 boot strap; do not make startup depend on holding it low through reset.
- Match the ESPHome screen: temperature in °C and relative humidity in %, one decimal place, labels `°C temp` and `% hum`, and `WARN!` when humidity is greater than or equal to the configured threshold. Store config should default to 65.0% to match `ESPHome-HumiTempSensor.yaml`. Support 0°/180° screen orientation. Pressure is measured and sent to the Store but is not shown by the ESPHome display.
- ESPHome dims by sun elevation. The sensor has no clock/location source, so match the content and warning first; add solar dimming only if the Store can supply a validated contrast value. Clamp contrast to the display's usable range. Show a clear `WAIT STORE` or `NO CONFIG` screen until initial configuration arrives.

## Store-owned configuration and startup

Do not persist operational configuration on the sensor. At boot, initialize the display and radio, identify the node by its station MAC, then request a complete configuration snapshot from the Store. Apply it in RAM and acknowledge its revision. Request again after Store reboot, timeout, or config-revision change. The snapshot should include sensor name, sample interval, humidity warning threshold, screen rotation, optional contrast, and protocol/config revision. Reject invalid values and keep retrying with a visible status rather than silently substituting stale settings.

The sensor still needs a small bootstrap identity to find and authenticate the Store: a discovery strategy (scan channels or use a fixed channel) and a unique per-sensor bootstrap key or equivalent credential. Store the matching key with that sensor's SD-card entry. This is identity material, not changeable operating config; without some credential or physical provisioning step, a sensor cannot securely retrieve its first config. Store-initiated physical pairing should authorize new MACs before adding them to the allowlist.

## Radio protocol, channel, and reliability

Use one shared, versioned protocol for both projects. Keep packets at or below 250 bytes for compatibility with ESP8266 ESP-NOW. Define fixed-width, explicitly encoded fields rather than sending raw C++ structs or floats. Include protocol version, message type, source/destination ID, transaction or sample sequence, config revision, and payload length. Message types should cover `HELLO/CONFIG_REQUEST`, `CONFIG_SNAPSHOT`, `ACK/STATUS`, `READ_REQUEST`, and `READING`; measurements should carry temperature (centi-degrees C), humidity (centi-percent), pressure (Pa), sample sequence, sensor boot ID/uptime, and validity flags. The boot ID and uptime let the Store estimate times for buffered readings after reconnect. Use application acknowledgements, bounded retries, duplicate suppression, and idempotent config updates; MAC-layer send success alone does not confirm the application processed a message.

Use encrypted unicast for known Store/sensor peers and reject unknown senders. Broadcast may be used only for discovery/pairing requests, with Store approval limited to a physical pairing window; never broadcast credentials. Provision a unique key per sensor instead of reusing one network-wide key. The Store and sensor share the same 2.4 GHz channel. The Store's single radio follows its connected Wi-Fi AP, so a sensor should periodically scan channels and retry discovery after a network/channel change. It continues displaying measurements while disconnected and keeps a small bounded RAM queue for catch-up readings; this queue is lost on power failure. Use a Store-configured interval, initially 10 seconds to match ESPHome.

## Implementation milestones and acceptance

1. Create the PlatformIO environment and pin the ESP8266 core, sensor/display libraries, and ESP-NOW API.
2. Implement I2C sampling, display formatting, config validation, startup/wait states, and warning/orientation behavior.
3. Implement the shared wire protocol, authenticated pairing/config fetch, channel recovery, acknowledged readings, and bounded buffering.
4. Compile for the selected board and verify on hardware: boot with/without Store, config refresh without sensor flash writes, invalid config rejection, display boundary at exactly the threshold, Store Wi-Fi channel change recovery, retries/duplicate packets, and readings received by the Store.

Keep credentials and keys out of examples and source control. Record chosen channel recovery interval, buffer size, library versions, and tested board/flash configuration when implementation begins.
