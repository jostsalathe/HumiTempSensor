#define SENSOR_TYPE_BME280
//#define SENSOR_TYPE_DHT22

#ifdef SENSOR_TYPE_BME280
#include <BME280I2C.h>
#endif
#ifdef SENSOR_TYPE_DHT22
#include <DHT.h>
#endif
#include <Wire.h>
#include <SSD1306Wire.h>
#include <ESP8266WiFi.h>
#include <ThingsBoard.h>
#include <ParametersSPIFFS.h>
#include <EspBootstrapDict.h>

const String TOKEN("HumiTempSensorV3"); //!< configuration version token to prevent misconfiguration after firmware upgrade

const int N_CONFIG = 12;                //!< number of configuration parameters including config page title
Dictionary _configuration(N_CONFIG);    //!< holds configuration during runtime 


// === pinout definitions =============================
const uint8_t LED_ST_0              =  4; //!< status LED pin (active HIGH)
const uint8_t LED_ST_1              =  5; //!< status LED pin (active HIGH)
const uint8_t GPIO12                = 12; //!< unused GPIO pin
const uint8_t GPIO13                = 13; //!< unused GPIO pin
const uint8_t DEBUG_EN              = 14; //!< debug enable pin / solder bridge (active LOW)
const uint8_t DISPLAY_SENSOR_DATA   =  2; //!< data pin for IIC (display and sensor)
const uint8_t DISPLAY_SENSOR_CLOCK  =  0; //!< clock pin for IIC (display and sensor)
const uint8_t FORCE_CFG             =  0; //!< force configuration AP pin / button (active LOW)

WiFiClient _wifiClient;
IPAddress _staticIP, _gateway, _subnet;

// === construct SSD1306 OLED. ========================
SSD1306Wire _display(0x3c, DISPLAY_SENSOR_DATA, DISPLAY_SENSOR_CLOCK, GEOMETRY_128_32);

// === construct sensor. ==============================
#ifdef SENSOR_TYPE_BME280
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
#endif

#ifdef SENSOR_TYPE_DHT22
#define SENSOR_PIN GPIO13
DHT sensor(SENSOR_PIN, DHT22);
#endif

// === construct ThingsBoard endpoint with 128 byte payload size and support for 3 Fields
ThingsBoardSized<128, 3> tb(_wifiClient); //!< endpoint for connection to ThingsBoard

bool _debug = true;       //!< whether debug output should be printed, or not (determined on reset by DEBUG_EN pin)
bool _stayAwake = true;   //!< whether the device should stay awake between measurements instead of deep sleeping
unsigned long _interval;  //!< measurement interval in seconds
float _warnThreshold;     //!< humidity threshold at which to show a warning on the display
unsigned long _nextMeasurement = 0; //!< time point of next measurement according to millis()



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
  Wire.begin(DISPLAY_SENSOR_DATA, DISPLAY_SENSOR_CLOCK);

  if (!readConfig()) runConfigAPAndRestart();

  startConnectWiFi();
  
  sensor.begin();
}

/**
 * @brief running the config AP if requested, measuring, reporting
 * @details if deep sleep is enabled, this should run exactly once and goes to deep sleep after that
 * 
 */
void loop()
{
  if (isForceConfig())
  {
    runConfigAPAndRestart();
  }
  else if (_nextMeasurement <= millis())
  {
    _nextMeasurement += _interval * 1000;
    
    getAndReportSensorData();

    if (!_stayAwake)
    {
      // disconnecting from ThingsBoard
      printDebug("Disconnecting from ThingsBoard...");
      tb.disconnect();

      // preparing deep sleep
      digitalWrite(LED_ST_0, LOW);
      digitalWrite(LED_ST_1, LOW);
      long sleepMs = _nextMeasurement - millis();
      if (sleepMs < 1) sleepMs = 1;
      printDebug("Entering deep sleep for " + String(sleepMs) + "ms...");
      ESP.deepSleep(static_cast<uint64_t>(sleepMs) * 1000);
    }
  }
}

/**
 * @brief checks, whether config AP shall be forced
 * @details Usage is: Release reset and then immediately press and hold Boot button
 * until status LEDs alternatingly flash at 1 Hz, then release boot button -> should start config AP
 * @return true, if FORCE_CFG (aka Boot button) pin is LOW (pressed), returns as soon as it is HIGH again
 * @return false, else
 */
bool isForceConfig()
{
  if (!digitalRead(FORCE_CFG))
  {
    logToSerial("forced config mode by pulling pin " + String(FORCE_CFG) + " low!");
    while(!digitalRead(FORCE_CFG))
    {
      // blinking LEDs alternatingly at 1 Hz
      if (millis()%1000<500)
      {
        digitalWrite(LED_ST_0, HIGH);
        digitalWrite(LED_ST_1, LOW);
      }
      else
      {
        digitalWrite(LED_ST_0, LOW);
        digitalWrite(LED_ST_1, HIGH);
      }
      delay(10);
    }
    digitalWrite(LED_ST_0, LOW);
    digitalWrite(LED_ST_1, LOW);
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
  printDebug("reading configuration files from Flash...");

  // filling the configuration dictionary with defaults
  _configuration("Title", "HumiTempSensor Configuration");
  _configuration("wifiSsid", "your WIFI SSID");
  _configuration("wifiPassword", "your WIFI password");
  _configuration("staticIP", "192.168.0.42");
  _configuration("gateway", "192.168.0.1");
  _configuration("subnet", "255.255.255.0");
  _configuration("thingsboardServer", "thingsboard-server.hostname");
  _configuration("thingsboardToken", "your thingsboard sensor token");
  _configuration("interval", "10");
  _configuration("stayAwake", "true");
  _configuration("warnThreshold", "65");
  _configuration("flipScreen", "false");

  // try loading configuration from Flash
  // LittleFS.begin();
  SPIFFS.begin();
  ParametersSPIFFS param(TOKEN, _configuration);
  param.begin();
  param.load();
  // LittleFS.end();
  SPIFFS.end();

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

  _stayAwake = _configuration["stayAwake"] == "true";

  _warnThreshold = _configuration["warnThreshold"].toFloat();
  if (_warnThreshold < 0.0f || _warnThreshold > 100.0f)
  {
    logToSerial("ERROR: couldn't convert warnThreshold(\"" + _configuration["warnThreshold"] + "\") to float >0 and <100");
    return false;
  }

  if (isForceConfig()) return false;

  printDebug("reading configuration files from Flash - done.");
  return true;
}

/**
 * @brief saves config to json file on chip module's flash
 * 
 */
void saveConfig()
{
  logToSerial("Write new configuration to Flash...");
  // LittleFS.begin();
  SPIFFS.begin();
  ParametersSPIFFS param(TOKEN, _configuration);
  param.begin();
  param.save();
  // LittleFS.end();
  SPIFFS.end();
  logToSerial("Write new configuration to Flash - done.");
}

/**
 * @brief runs access point so WiFi settings can be made and restarts the Processor afterwards
 * 
 */
void runConfigAPAndRestart()
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
  if (ESPBootstrap.run(_configuration, N_CONFIG - 1, 10 * BOOTSTRAP_MINUTE) == BOOTSTRAP_OK)
  {
    logToSerial(String("Received new configuration: ") + _configuration.json());
    saveConfig();
  }
  else
  {
    logToSerial("No new configuration received before timeout.");
  }

  logToSerial("Getting new configuration - done.");
  delay(5000);
  logToSerial("Restarting device...");
  ESP.restart();
}

/**
 * @brief Core function. Gets data, sends data and displays data
 * 
 */
void getAndReportSensorData()
{
  printDebug("Acquire new sensor data...");
  // pressure in hPa
  float pressure = NAN;
  // temperature in Celsius
  float temperature = NAN;
  // humidity in percent
  float humidity = NAN;

  // read sensor
  if (isForceConfig()) runConfigAPAndRestart();
#ifdef SENSOR_TYPE_BME280
  sensor.read(pressure, temperature, humidity, BME280I2C::TempUnit_Celsius, BME280I2C::PresUnit_hPa);
  if (isForceConfig()) runConfigAPAndRestart();
  delay(100);
  if (isForceConfig()) runConfigAPAndRestart();
  sensor.read(pressure, temperature, humidity, BME280I2C::TempUnit_Celsius, BME280I2C::PresUnit_hPa);
#endif
#ifdef SENSOR_TYPE_DHT22
  temperature = sensor.readTemperature();
  sensor.read();
  humidity = sensor.readHumidity();
#endif
  if (isForceConfig()) runConfigAPAndRestart();
  printDebug("Measured: "
#ifdef SENSOR_TYPE_BME280
             "pres=" + String(pressure,2) + "hPa, "
#endif
             "temp=" + String(temperature,2) + "°C, "
             "hum=" + String(humidity,2) + "%");

  if (isForceConfig()) runConfigAPAndRestart();
  displayMeasurements(humidity, temperature, pressure);
  if (isForceConfig()) runConfigAPAndRestart();
  reportMeasurements(humidity, temperature, pressure);
  if (isForceConfig()) runConfigAPAndRestart();
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
 */
void reportMeasurements(float hum, float temp, float pres)
{
  printDebug("Transmit to server...");
  
  if (isnan(hum)
      || isnan(temp)
#ifdef SENSOR_TYPE_BME280
      || isnan(pres)
#endif
  )
  {
    printDebug("ERROR: Sensor read failed, faulty data not transmitted!");
    return;
  }

  // wait for WiFi connection
  if (!waitConnectWiFi()) return;

  // connecting to ThingsBoard
  if (_debug) digitalWrite(LED_ST_1, tb.connected());

  if (!tb.connected())
  {
    printDebug("Connecting to ThingsBoard...");
    // need to connect
    if (!tb.connect(_configuration["thingsboardServer"].c_str(), _configuration["thingsboardToken"].c_str()))
    {
      // could not connect
      printDebug("Connecting to ThingsBoard failed!");
      if (_debug) digitalWrite(LED_ST_1, tb.connected());
      return;
    }
  }

  printDebug("Sending data to ThingsBoard...");
  // sending data to ThingsBoard
  const int DATA_ITEMS = 3;
  Telemetry data[DATA_ITEMS] =
  {
    {"temperature", temp},
    {"humidity",    hum}
#ifdef SENSOR_TYPE_BME280
    ,{"pressure",    pres}
#endif
  };
  tb.sendTelemetry(data, DATA_ITEMS);

  if (_debug) digitalWrite(LED_ST_1, tb.connected());

  printDebug("Transmit to server - done.");
  return;
}

/**
 * @brief initiates WiFi connection with several retries, if necessary
 * 
 */
void startConnectWiFi()
{
  printDebug("Connecting to WiFi...");
  if (_debug) digitalWrite(LED_ST_0, WiFi.status() == WL_CONNECTED);

  // attempt to connect to WiFi network
  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  WiFi.config(_staticIP, _gateway, _subnet);
  WiFi.begin(_configuration["wifiSsid"].c_str(), _configuration["wifiPassword"].c_str());
}

bool waitConnectWiFi()
{
  // immediately return if already connected
  if (WiFi.status() == WL_CONNECTED) return true;

  int remainigRetries = 3;
  printDebug("Wait for WiFi...");
  const unsigned long TIMEOUT = 5000;
  unsigned long timeoutEnd = millis() + TIMEOUT;

  // wait until WiFi is connected
  while (WiFi.status() != WL_CONNECTED)
  {
    if (_debug)
    {
      // flash LED_ST_0 with 1 Hz
      if (millis() % 1000 < 100) digitalWrite(LED_ST_0, HIGH);
      else digitalWrite(LED_ST_0, LOW);
    }
    // try a reconnect if necessary and still wanted
    if (WiFi.status() == WL_CONNECT_FAILED && remainigRetries > 0)
    {
      WiFi.reconnect();

      logToSerial("WiFi connect failed! Retry...");
      --remainigRetries;
      timeoutEnd = millis() + TIMEOUT;
    }

    // last try to reconnect is too long ago - just give up, already
    if (millis() > timeoutEnd)
    {
      logToSerial("WiFi timeout with WiFi.status(" + String(WiFi.status()) + ")!");
      digitalWrite(LED_ST_0, LOW);
      return false;
    }

    // run the configuration AP if forced by button press
    if (isForceConfig()) runConfigAPAndRestart();
    
    delay(10);
  }

  if (_debug) digitalWrite(LED_ST_0, HIGH);
  if (_debug) Serial.setDebugOutput(false);
  printDebug("Connecting to WiFi - done.");
  return true;
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
