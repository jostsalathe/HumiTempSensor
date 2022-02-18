# HumiTempSensor
Firmware for little board around an ESP-12F using a BME280 to record temperature and rel. humidity to ThingsBoard.

## What you need
You need the following software components:
- Arduino IDE
- [esp8266 board package](https://github.com/esp8266/Arduino)
- [ThingsBoard Arduino MQTT SDK](https://github.com/thingsboard/ThingsBoard-Arduino-MQTT-SDK)
- [ThingPulse esp8266 ssd1306 oled library](https://github.com/ThingPulse/esp8266-oled-ssd1306)
- [BME280 library by Tyler Glenn](https://github.com/finitespace/BME280)
- [Dictionary library by Anatoli Arkhipenko](https://github.com/arkhipenko/Dictionary)
- [EspBootstrap library by Anatoli Arkhipenko](https://github.com/arkhipenko/EspBootstrap)
- [ThingsBoard server](https://thingsboard.io/) where the sensor can post its data and where you can view that data on dashboards
- A token for the sensor that is registered in your ThingsBoard

You need the following hardware components:
- ESP12 board or compatible including USB programming adapter (e.g. NodeMCU)
- BME280 sensor
- SSD1306 128*32 OLED display module
- power source
- A WiFi network that the sensor can connect to

I built a nice little board ([ESP8266 12F BME280](https://easyeda.com/jostsalathe/ESP8266-12e-Base-Board_copy)) that has everything except the programmer on board. For uploading firmware to this board I use a USB FTDI232 adapter.

I recommend using some wire to mount the sensor with a few centimeters distance to the main board since the ESP gets a little warm even when only sampling every 30 second.

## How to use it
- Clone this repository
- Upload the sketch via the *Arduino* IDE
- Connect to WiFi access point created by the ESP and open http://10.1.1.1
- Insert your configuration data and confirm
- The ESP should restart and, if configured correctly, report its measurements regularly from now on
- If it encounters an error, it will open a configuration access point, again
- You can always force configuration mode by pulling pin 0 low after reset (e.g. by pressing the boot mode button)
- Baud rate is 74880
