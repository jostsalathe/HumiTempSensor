#include "ESP8266WiFi.h"
#include <espnow.h>

#include <BME280I2C.h>
#include <Wire.h>
#include <SSD1306Wire.h>


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

// === pinout definitions =============================
const uint8_t LED_ST_0              =  4; //!< status LED pin (active HIGH)
const uint8_t LED_ST_1              =  5; //!< status LED pin (active HIGH)
const uint8_t GPIO12                = 12; //!< unused GPIO pin
const uint8_t GPIO13                = 13; //!< unused GPIO pin
const uint8_t DEBUG_EN              = 14; //!< debug enable pin / solder bridge (active LOW)
const uint8_t DISPLAY_SENSOR_DATA   =  2; //!< data pin for IIC (display and sensor)
const uint8_t DISPLAY_SENSOR_CLOCK  =  0; //!< clock pin for IIC (display and sensor)
const uint8_t FORCE_CFG             =  0; //!< force configuration AP pin / button (active LOW)

// === construct SSD1306 OLED. ========================
SSD1306Wire _display(0x3c, DISPLAY_SENSOR_DATA, DISPLAY_SENSOR_CLOCK, GEOMETRY_128_32);

// === construct sensor. ==============================
BME280I2C::Settings _bmeSettings(
  BME280I2C::OSR_X1,              // temp oversampling
  BME280I2C::OSR_X1,              // humidity oversampling
  BME280I2C::OSR_X1,              // pressure oversampling
  BME280I2C::Mode_Forced,
  BME280I2C::StandbyTime_1000ms,
  BME280I2C::Filter_Off,
  BME280I2C::SpiEnable_False,
  (BME280I2C::I2CAddr)0x76        // I2C address. (0x76 is default, 0x77 alternative)
);
BME280I2C sensor(_bmeSettings);

// === strapping pin defined config ===================
bool _debug = true; //!< whether debug output should be printed, or not (determined on reset by DEBUG_EN pin)

// === EspNowStore defined config =====================
bool _flipScreen = false;           //!< whether the display content should be rotated 180°
float _warnThreshold = 70.0f;       //!< humidity threshold at which to show a warning on the display

// === global working variables =======================
unsigned long _nextMeasurement = 0; //!< time point of next measurement according to millis()
float _lastPressure = NAN;          //!< in hPa, last measurement value for pressure
float _lastTemperature = NAN;       //!< in Celsius, last measurement value for temperature
float _lastHumidity = NAN;          //!< in percent relative, last measurement value for humidity


/**
 * @brief init of entire program
 * 
 */
void setup()
{
  pinMode(DEBUG_EN, INPUT_PULLUP);
  // internally pulled high - solder bridge open
  _debug = !digitalRead(DEBUG_EN);
  pinMode(LED_ST_0, OUTPUT);
  pinMode(LED_ST_1, OUTPUT);
  digitalWrite(LED_ST_0, LOW);
  digitalWrite(LED_ST_1, LOW);

  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting HumiTempSensor");
  Wire.begin(DISPLAY_SENSOR_DATA, DISPLAY_SENSOR_CLOCK);

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
  sensor.begin();
}

/**
 * @brief measuring and displaying
 * 
 */
void loop()
{
  if (_nextMeasurement <= millis())
  {
    _nextMeasurement += 1000;
    getAndShowSensorData();
  }
  delay(1);
}


/**
 * @brief Core function. Gets data and displays data
 * 
 */
void getAndReportSensorData()
{
  float dummy;
  if(_debug) digitalWrite(LED_ST_0, HIGH);
  printDebug("Acquire new sensor data...");
  sensor.read(dummy, dummy, dummy, BME280I2C::TempUnit_Celsius, BME280I2C::PresUnit_hPa);
  delay(100);
  sensor.read(_lastPressure, _lastTemperature, _lastHumidity, BME280I2C::TempUnit_Celsius, BME280I2C::PresUnit_hPa);
  printDebug("Measured: pres=" + String(_lastPressure,2)
              + "hPa, temp=" + String(_lastTemperature,2)
              + "°C, hum=" + String(_lastHumidity,2) + "%");
  displayMeasurements(humidity, temperature, pressure);
  digitalWrite(LED_ST_0, LOW);
}


/**
 * @brief writes values to display
 * 
 * @param hum relative humidity in %
 * @param temp temperature in °C
 * @param pres pressure is in hPa
 */
void displayMeasurements(float hum, float temp, float pres)
{
  printDebug("Display data...");
  _display.init();
  if (_configuration["flipScreen"] == "true") _display.flipScreenVertically();
  _display.setBrightness(1);
  _display.setTextAlignment(TEXT_ALIGN_LEFT);
  _display.clear();

  String sTemp(temp,1);
  if (temp < 10.0f) sTemp = "0" + sTemp;

  String sHum(hum,1);
  if (hum >= 100.0f) sHum = "99.9";
  if (hum < 10.0f) sHum = "0" + sHum;
  
  _display.setFont(ArialMT_Plain_24);
  int wHum = _display.getStringWidth(sHum);
  int wTemp = _display.getStringWidth(sTemp);
  _display.drawString(2, 9, sTemp);
  _display.drawString(125-wHum, 9, sHum);

  _display.setFont(ArialMT_Plain_10);
  String sHumText = "% hum";
  String sTempText = "°C temp";
  int wHumText = _display.getStringWidth(sHumText);
  int wTempText = _display.getStringWidth(sTempText);
  _display.drawString(4, 0, sTempText);
  _display.drawString(123-wHumText, 0, sHumText);

  if (hum>_warnThreshold)
  {
    _display.setFont(ArialMT_Plain_10);
    String sWarn = "WARN!";
    int wWarn = _display.getStringWidth(sWarn);
    int pWarn = (127-wTempText-wHumText)/2 + wTempText - wWarn/2;
    _display.drawString(pWarn, 0, sWarn);
    _display.invertDisplay();
  }
  else
  {
    _display.normalDisplay();
  }
  
  _display.display();
  printDebug("Display data - done.");
}



/**
 * @brief callback for reception events
 * 
 * @param mac String containing recipients mac address
 * @param data String containing received data
 * @param len length of received data
 */
void espNowRxCb(const uint8_t* mac, const uint8_t* data, uint8_t len)
{
  //TODO: receive config after pairing
  //TODO: receive data request (NAK if configuration not present)
}

/**
 * @brief callback for transmission events
 * 
 * @param mac String containing recipients mac address
 * @param status transmission result code
 */
void espNowTxCb(const uint8_t* mac, uint8_t status)
{
  //TODO: send latest measurements once if unhandled data request has been received
}

/**
 * @brief logs to Serial whilst adding a time stamp
 * 
 * @param logTxt String to print to Serial
 * @sa Serial.println()
 */
void logToSerial(const String & logTxt)
{
  unsigned long time = millis();
  int milliseconds = time % 1000;
  int seconds = (time / 1000) % 60;
  int minutes = (time / 1000 / 60) % 60;
  int hours = time / 1000 / 60 / 60;
  char buf[32];
  sprintf(buf,"%4d:%02d:%02d.%03d ", hours, minutes, seconds, milliseconds);
  Serial.print(buf);
  Serial.println(logTxt);
}

/**
 * @brief convenience wrapper to only logToSerial, when _debug is set
 *
 * @param debugTxt String to print to Serial
 * @sa logToSerial()
 */
void printDebug(const String & debugTxt)
{
  if (_debug) logToSerial(debugTxt);
}
