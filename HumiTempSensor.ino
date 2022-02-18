#include <BME280I2C.h>
#include <Wire.h>
#include <SSD1306Wire.h>
#include <ESP8266WiFi.h>
#include <ThingsBoard.h>
#include <LittleFS.h>
#include <EspBootstrapDict.h>


const String TOKEN("HumiTempSensorV2");

const int NCONFIG = 11;
Dictionary _configuration(NCONFIG);


// === pinout definitions =============================
#define LED_ST_0 4
#define LED_ST_1 5
#define GPIO12 12
#define GPIO13 13
#define DEBUG_EN 14
#define SDA 2
#define SCL 0
#define FORCE_CFG 0

WiFiClient _wifiClient;
IPAddress _staticIP, _gateway, _subnet;

// === construct SSD1306 OLED. ========================
SSD1306Wire _display(0x3c, SDA, SCL, GEOMETRY_128_32);

// === construct BME280I2C sensor.
BME280I2C::Settings _bmeSettings(
  BME280I2C::OSR_X1, // temp oversampling
  BME280I2C::OSR_X1, // humidity oversampling
  BME280I2C::OSR_X1, // pressure oversampling
  BME280I2C::Mode_Forced,
  BME280I2C::StandbyTime_1000ms,
  BME280I2C::Filter_Off,
  BME280I2C::SpiEnable_False,
  (BME280I2C::I2CAddr)0x76 // I2C address. (0x76 is default, 0x77 alternative)
);
BME280I2C bme(_bmeSettings);

bool _debug = true;      //!< whether debug output should be printed, or not

unsigned int _interval; //!< measurement interval

float _warnThreshold;   //!< when to output a warning

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
  
  Serial.begin(74880);
  Wire.begin(SDA, SCL);

  if (readConfig()) getAndReportSensorDataThenSleep();

  runConfigAP();
}

/**
 * @brief nothing to do here.... yet
 * 
 */
void loop()
{
}

/**
 * @brief checks, whether config shall be forced
 * 
 * @return true, if FORCE_CFG pin is HIGH
 * @return false, else
 */
bool isForceConfig()
{
  if (!digitalRead(FORCE_CFG))
  {
    logToSerial("forced config mode by pulling pin " + String(FORCE_CFG) + " low!");
    return true;
  }
  return false;
}

/**
 * @brief read config from module's flash
 * 
 * @return true, if settings were made and seem valid
 * @return false, if settings files either does not exist or seems invalid
 */
bool readConfig()
{
  printDebug("reading configuration files from SPIFFS...");

  if (LittleFS.exists("/config.json"))
  {
    Serial.println("reading config file");
    File configFile = LittleFS.open("/config.json", "r");
    if (configFile)
    {
      Serial.println("opened config file");
      DynamicJsonDocument jsonDoc(2048);
      auto error = deserializeJson(jsonDoc, configFile);
      serializeJsonPretty(jsonDoc, Serial);
      if (!error)
      {
        Serial.println("\nparsed json");
        _configuration("Title", "HumiTempSensor Configuration");
        _configuration("wifiSsid", "your WIFI SSID");
        _configuration("wifiPassword", "your WIFI password");
        _configuration("staticIP", "192.168.0.42");
        _configuration("gateway", "192.168.0.1");
        _configuration("subnet", "255.255.255.0");
        _configuration("thingsboardServer", "thingsboard-server.hostname");
        _configuration("thingsboardToken", "your thingsboard sensor token");
        _configuration("interval", "10");
        _configuration("warnThreshold", "65");
        _configuration("flipScreen", "false");
      }
      else
      {
        Serial.print("failed to load json config with code ");
        Serial.print(error.c_str());
        Serial.println(" - using default configuration.");
      }
    }
    else
    {
      Serial.println("failed to open /config.json - using default configuration.");
    }
    configFile.close();
  }

  if (!_staticIP.fromString(_configuration["staticIP"]))
  {
    logToSerial("ERROR: couldn't convert staticIP(\"" + _configuration["staticIP"] + "\") to IP address");
    return false;
  }

  if (!_gateway.fromString(_configuration["gateway"]))
  {
    logToSerial("ERROR: couldn't convert gateway(\"" + _configuration["gateway"] + "\") to IP address");
    return false;
  }

  if (!_subnet.fromString(_configuration["subnet"]))
  {
    logToSerial("ERROR: couldn't convert subnet(\"" + _configuration["subnet"] + "\") to IP address");
    return false;
  }
  
  _interval = _configuration["interval"].toInt();
  if (_interval <= 0)
  {
    logToSerial("ERROR: couldn't convert interval(\"" + _configuration["interval"] + "\") to integer >0");
    return false;
  }

  _warnThreshold = _configuration["warnThreshold"].toFloat();
  if (_warnThreshold < 0.0f || _warnThreshold > 100.0f)
  {
    logToSerial("ERROR: couldn't convert warnThreshold(\"" + _configuration["warnThreshold"] + "\") to float >0 and <100");
    return false;
  }

  if (isForceConfig()) return false;

  printDebug("reading configuration files from SPIFFS - done.");
  return true;
}

/**
 * @brief saves config to json file on chip module's flash
 * 
 */
void saveConfig()
{
  Serial.println("writing config file");
  File configFile = LittleFS.open("/config.json", "w");
  if (configFile)
  {
    Serial.println("opened config file");
    DynamicJsonDocument jsonDoc(2048);
    jsonDoc["Title"] = _configuration("Title");
    jsonDoc["wifiSsid"] = _configuration("wifiSsid");
    jsonDoc["wifiPassword"] = _configuration("wifiPassword");
    jsonDoc["staticIP"] = _configuration("staticIP");
    jsonDoc["gateway"] = _configuration("gateway");
    jsonDoc["subnet"] = _configuration("subnet");
    jsonDoc["thingsboardServer"] = _configuration("thingsboardServer");
    jsonDoc["thingsboardToken"] = _configuration("thingsboardToken");
    jsonDoc["interval"] = _configuration("interval");
    jsonDoc["warnThreshold"] = _configuration("warnThreshold");
    jsonDoc["flipScreen"] = _configuration("flipScreen");
    serializeJsonPretty(jsonDoc, Serial);
    Serial.println("\nfilled json");
    serializeJson(jsonDoc, configFile);
    Serial.println("wrote config file");
  }
  else
  {
    Serial.println("failed to open /config.json.");
  }
  configFile.close();
}

/**
 * @brief runs access point so WiFi settings can be made
 * 
 */
void runConfigAP()
{
  logToSerial("Getting new configuration...");
  _display.init();
  if (_configuration["flipScreen"] == "true") _display.flipScreenVertically();
  _display.setBrightness(1);
  _display.setTextAlignment(TEXT_ALIGN_LEFT);
  _display.setFont(ArialMT_Plain_10);
  _display.clear();

  String ssid(SSID_PREFIX);
  ssid += WiFi.macAddress();
  ssid.replace(":", "");
  ssid.toLowerCase();

  logToSerial("Print message to display...");
  _display.drawString(0, 0, "Edit configuration on SSID");
  _display.drawString(0, 11, ssid);
  _display.drawString(0, 22, "browse http://10.1.1.1");
  
  _display.normalDisplay();
  _display.display();
  logToSerial("Print message to display - done.");

  logToSerial(String("Starting access point with SSID \"") + ssid + String("\""));
  logToSerial("Open http://10.1.1.1 in your browser to configure your device.");
  if (ESPBootstrap.run(_configuration, NCONFIG-1, 10 * BOOTSTRAP_MINUTE) == BOOTSTRAP_OK)
  {
    logToSerial(String("Received new configuration: ") + _configuration.json());
    logToSerial("Write new configuration to SPIFFS...");
    saveConfig();
    logToSerial("Write new configuration to SPIFFS - done.");
  }

  logToSerial("Getting new configuration - done.");
  delay(5000);
  logToSerial("Restarting device...");
  ESP.restart();
}

/**
 * @brief core function. gets data, sends data, displays data and then goes sleeping
 * 
 */
void getAndReportSensorDataThenSleep()
{
  startConnectWiFi();
  
  bme.begin();
  
  printDebug("Acquire new sensor data...");
  // pressure in hPa
  float pressure = NAN;
  // temperature in Celsius
  float temperature = NAN;
  // humidity in percent
  float humidity = NAN;

  // read sensor
  if (isForceConfig()) return;
  bme.read(pressure, temperature, humidity, BME280I2C::TempUnit_Celsius, BME280I2C::PresUnit_hPa);
  if (isForceConfig()) return;
  delay(100);
  if (isForceConfig()) return;
  bme.read(pressure, temperature, humidity, BME280I2C::TempUnit_Celsius, BME280I2C::PresUnit_hPa);
  if (isForceConfig()) return;
  printDebug("Measured: pres=" + String(pressure,2) + "hPa, temp=" + String(temperature,2) + "°C, hum=" + String(humidity,2) + "%");

  if (isForceConfig()) return;
  displayMeasurements(humidity, temperature, pressure);
  if (isForceConfig()) return;
  if (!reportMeasurements(humidity, temperature, pressure)) return;
  if (isForceConfig()) return;
  
  long sleepMs = (_interval * 1000) - millis();
  if (sleepMs < 1) sleepMs = 1;
  printDebug("Entering deep sleep for " + String(sleepMs) + "ms...");
  ESP.deepSleep(static_cast<uint64_t>(sleepMs) * 1000);
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
 * @brief sends data to Things board
 * 
 * @param hum relative humidity in %
 * @param temp temperature in °C
 * @param pres pressure is in hPa
 * @return true, always
 * @return false, never
 */
bool reportMeasurements(float hum, float temp, float pres)
{
  printDebug("Transmit to server...");
  
  if (isnan(hum) || isnan(temp) || isnan(pres))
  {
    printDebug("ERROR: Sensor read failed, faulty data not transmitted!");
    return true;
  }

  // wait for WiFi connection
  if (!waitConnectWiFi()) return true;

  // connecting to ThingsBoard
  ThingsBoard tb(_wifiClient);
  printDebug("Connecting to ThingsBoard...");
  if (_debug) digitalWrite(LED_ST_1, HIGH);

  if (tb.connect(_configuration["thingsboardServer"].c_str(), _configuration["thingsboardToken"].c_str()))
  {
    printDebug("Sending data to ThingsBoard...");
    // sending data to ThingsBoard
    const int DATA_ITEMS = 3;
    Telemetry data[DATA_ITEMS] =
    {
      {"temperature", temp},
      {"humidity",    hum},
      {"pressure",    pres}
    };
    tb.sendTelemetry(data, DATA_ITEMS);
  }
  else
  {
    printDebug("Connecting to ThingsBoard failed!");
    if (_debug) digitalWrite(LED_ST_1, LOW);
  }

  // disconnecting from ThingsBoard
  printDebug("Disconnecting from ThingsBoard...");
  tb.disconnect();
  digitalWrite(LED_ST_1, LOW);
  
  // disconnecting from WiFi
  disconnectWiFi();
  
  printDebug("Transmit to server - done.");
  return true;
}

/**
 * @brief initiates WiFi connection with several retries, if necessary
 * 
 */
void startConnectWiFi()
{
  printDebug("Connecting to WiFi...");
  if (_debug) digitalWrite(LED_ST_0, HIGH);

  // attempt to connect to WiFi network
  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  WiFi.config(_staticIP, _gateway, _subnet);
  WiFi.begin(_configuration["wifiSsid"].c_str(), _configuration["wifiPassword"].c_str());
}

bool waitConnectWiFi()
{
  int remainigRetries = 3;
  printDebug("Wait for WiFi...");
  const unsigned long TIMEOUT = 5000;
  unsigned long timeoutEnd = millis() + TIMEOUT;

  while (WiFi.status() != WL_CONNECTED)
  {
    if (WiFi.status() == WL_CONNECT_FAILED && remainigRetries > 0)
    {
      WiFi.reconnect();

      logToSerial("WiFi connect failed! Retry...");
      --remainigRetries;
      timeoutEnd = millis() + TIMEOUT;
    }

    if (millis() > timeoutEnd)
    {
      logToSerial("WiFi timeout with WiFi.status(" + String(WiFi.status()) + ")!");
      return false;
    }

    if (isForceConfig()) return false;
    
    delay(10);
  }

  if (_debug) Serial.setDebugOutput(false);
  printDebug("Connecting to WiFi - done.");
  return true;
}

/**
 * @brief should disconnect from WiFi, but currently does not. Pulls down LED_ST_0
 * 
 */
void disconnectWiFi()
{
  // printDebug("Disconnecting from WiFi.");
  // WiFi.disconnect(true);
  // delay(1);
  digitalWrite(LED_ST_0, LOW);
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
