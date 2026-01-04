#include "ESP8266WiFi.h"
#include <espnow.h>

#include <BME280I2C.h>


/**
 * ESP-NOW COMMANDS {uint8_t cmd, ...}
 *  0x00 - set sensor config
 *   {uint8_t warnThreshold, uint8_t flipScreen}
 *  0x01 - request measurement data
 *   {}
 *  0x02 - send measurement data
 *   {float pressure, float temperature, float humidity}
 * CONFIG PARAMETERS (received from EspNowStore, not stored permanently on this device)
 *  StoreMac - mac address of EspNowStore device
 *  warnThreshold - humidity threshold at and above wich the display will invert for visible warning
 *  flipScreen - rotate the display content 180° to adapt to device orientation
 */

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting HumiTempSensor");

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.disconnect();
  WiFi.softAP("ESPNOW", nullptr, 3);
  WiFi.softAPdisconnect(false);
  // WiFi must be powered on to use ESP-NOW unicast.
  // It could be either AP or STA mode, and does not have to be connected.
  // For best results, ensure both devices are using the same WiFi channel.

  Serial.print("MAC address of this node is ");
  Serial.println(WiFi.softAPmacAddress());

  bool ok =
    esp_now_init() == 0 &&
#ifdef ARDUINO_ARCH_ESP8266
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO) == 0 &&
#endif
    esp_now_register_recv_cb(reinterpret_cast<esp_now_recv_cb_t>(espNowRxCb)) == 0 &&
    esp_now_register_send_cb(reinterpret_cast<esp_now_send_cb_t>(espNowTxCb)) == 0;
  if (!ok) {
    Serial.println("WifiEspNow.begin() failed");
    delay(1000);
    ESP.restart();
  }

  ok = WifiEspNow.addPeer(PEER);
  if (!ok) {
    Serial.println("WifiEspNow.addPeer() failed");
    ESP.restart();
  }
}

void loop()
{
  ;
}


void espNowRxCb(const uint8_t* mac, const uint8_t* data, uint8_t len)
{
  ;
}

void espNowTxCb(const uint8_t* mac, uint8_t status)
{
  ;
}
