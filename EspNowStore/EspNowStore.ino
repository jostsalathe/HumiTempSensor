#include "esp_wifi.h"
#include "esp_now.h"

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting HumiTempStore");

  //TODO: read config from SD card
  //TODO: fill list of known Sensors

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  // WiFi must be powered on to use ESP-NOW unicast.
  // This node wants to use a WiFi network for getting NTP time from the Internet
  //TODO: connect to WiFi specified on SD card

  bool ok =
    esp_now_init() == 0 &&
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
  //TODO: at some interval try pairing known sensors that are no peers, yet
  //TODO:  this includes sending associated Sensor config to the new peers
  //TODO: at some interval request sensor readings from all sensors
}


void espNowRxCb(const uint8_t* mac, const uint8_t* data, uint8_t len)
{
  ;
}

void espNowTxCb(const uint8_t* mac, uint8_t status)
{
  //TODO:  remove peers that failed to respond for later reconnect
}
