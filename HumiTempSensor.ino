#include <BME280I2C.h>
#include <Wire.h>
#include <SSD1306Wire.h>
#include <ESP8266WiFi.h>
#include <ThingsBoard.h>
#include <LittleFS.h>
#include <EspBootstrapDict.h>


const String TOKEN("HumiTempSensorV2");

const int NCONFIG = 11;
Dictionary configuration(NCONFIG);


// === pinout definitions =============================
#define LED_ST_0 4
#define LED_ST_1 5
#define GPIO12 12
#define GPIO13 13
#define DEBUG_EN 14
#define SDA 2
#define SCL 0
#define FORCE_CFG 0

WiFiClient wifiClient;
IPAddress staticIP, gateway, subnet;

// === construct SSD1306 OLED. ========================
SSD1306Wire display(0x3c, SDA, SCL, GEOMETRY_128_32);

// === construct BME280I2C sensor.
BME280I2C::Settings bmeSettings(
  BME280I2C::OSR_X1, // temp oversampling
  BME280I2C::OSR_X1, // humidity oversampling
  BME280I2C::OSR_X1, // pressure oversampling
  BME280I2C::Mode_Forced,
  BME280I2C::StandbyTime_1000ms,
  BME280I2C::Filter_Off,
  BME280I2C::SpiEnable_False,
  (BME280I2C::I2CAddr)0x76 // I2C address. (0x76 is default, 0x77 alternative)
);
BME280I2C bme(bmeSettings);

bool debug = true;

unsigned int interval;

float warnThreshold;



void setup()
{
  pinMode(DEBUG_EN, INPUT_PULLUP);
  // internally pulled high - solder bridge open
  debug = !digitalRead(DEBUG_EN);
  pinMode(LED_ST_0, OUTPUT);
  pinMode(LED_ST_1, OUTPUT);
  digitalWrite(LED_ST_0, LOW);
  digitalWrite(LED_ST_1, LOW);
  
  Serial.begin(74880);
  Wire.begin(SDA, SCL);

  if (readConfig()) getAndReportSensorDataThenSleep();

  runConfigAP();
}

void loop()
{
}

bool isForceConfig()
{
  if (!digitalRead(FORCE_CFG))
  {
    printError("forced config mode by pulling pin " + String(FORCE_CFG) + " low!");
    return true;
  }
  return false;
}

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
        configuration("Title", "HumiTempSensor Configuration");
        configuration("wifiSsid", "your WIFI SSID");
        configuration("wifiPassword", "your WIFI password");
        configuration("staticIP", "192.168.0.42");
        configuration("gateway", "192.168.0.1");
        configuration("subnet", "255.255.255.0");
        configuration("thingsboardServer", "thingsboard-server.hostname");
        configuration("thingsboardToken", "your thingsboard sensor token");
        configuration("interval", "10");
        configuration("warnThreshold", "65");
        configuration("flipScreen", "false");
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

  if (!staticIP.fromString(configuration["staticIP"]))
  {
    printError("ERROR: couldn't convert staticIP(\"" + configuration["staticIP"] + "\") to IP address");
    return false;
  }

  if (!gateway.fromString(configuration["gateway"]))
  {
    printError("ERROR: couldn't convert gateway(\"" + configuration["gateway"] + "\") to IP address");
    return false;
  }

  if (!subnet.fromString(configuration["subnet"]))
  {
    printError("ERROR: couldn't convert subnet(\"" + configuration["subnet"] + "\") to IP address");
    return false;
  }
  
  interval = configuration["interval"].toInt();
  if (interval <= 0)
  {
    printError("ERROR: couldn't convert interval(\"" + configuration["interval"] + "\") to integer >0");
    return false;
  }

  warnThreshold = configuration["warnThreshold"].toFloat();
  if (warnThreshold < 0.0f || warnThreshold > 100.0f)
  {
    printError("ERROR: couldn't convert warnThreshold(\"" + configuration["warnThreshold"] + "\") to float >0 and <100");
    return false;
  }

  if (isForceConfig()) return false;

  printDebug("reading configuration files from SPIFFS - done.");
  return true;
}

void saveConfig()
{
  Serial.println("writing config file");
  File configFile = LittleFS.open("/config.json", "w");
  if (configFile)
  {
    Serial.println("opened config file");
    DynamicJsonDocument jsonDoc(2048);
    jsonDoc["Title"] = configuration("Title");
    jsonDoc["wifiSsid"] = configuration("wifiSsid");
    jsonDoc["wifiPassword"] = configuration("wifiPassword");
    jsonDoc["staticIP"] = configuration("staticIP");
    jsonDoc["gateway"] = configuration("gateway");
    jsonDoc["subnet"] = configuration("subnet");
    jsonDoc["thingsboardServer"] = configuration("thingsboardServer");
    jsonDoc["thingsboardToken"] = configuration("thingsboardToken");
    jsonDoc["interval"] = configuration("interval");
    jsonDoc["warnThreshold"] = configuration("warnThreshold");
    jsonDoc["flipScreen"] = configuration("flipScreen");
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

void runConfigAP()
{
  printError("Getting new configuration...");
  display.init();
  if (configuration["flipScreen"] == "true") display.flipScreenVertically();
  display.setBrightness(1);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.clear();

  String ssid(SSID_PREFIX);
  ssid += WiFi.macAddress();
  ssid.replace(":", "");
  ssid.toLowerCase();

  printError("Print message to display...");
  display.drawString(0, 0, "Edit configuration on SSID");
  display.drawString(0, 11, ssid);
  display.drawString(0, 22, "browse http://10.1.1.1");
  
  display.normalDisplay();
  display.display();
  printError("Print message to display - done.");

  printError(String("Starting access point with SSID \"") + ssid + String("\""));
  printError("Open http://10.1.1.1 in your browser to configure your device.");
  if (ESPBootstrap.run(configuration, NCONFIG-1, 10 * BOOTSTRAP_MINUTE) == BOOTSTRAP_OK)
  {
    printError(String("Received new configuration: ") + configuration.json());
    printError("Write new configuration to SPIFFS...");
    saveConfig();
    printError("Write new configuration to SPIFFS - done.");
  }

  printError("Getting new configuration - done.");
  delay(5000);
  printError("Restarting device...");
  ESP.restart();
}

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
  
  long sleepMs = (interval * 1000) - millis();
  if (sleepMs < 1) sleepMs = 1;
  printDebug("Entering deep sleep for " + String(sleepMs) + "ms...");
  ESP.deepSleep(static_cast<uint64_t>(sleepMs) * 1000);
}

void displayMeasurements(float hum, float temp, float pres)
{
  printDebug("Display data...");
  display.init();
  if (configuration["flipScreen"] == "true") display.flipScreenVertically();
  display.setBrightness(1);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.clear();

  String sTemp(temp,1);
  if (temp < 10.0f) sTemp = "0" + sTemp;

  String sHum(hum,1);
  if (hum >= 100.0f) sHum = "99.9";
  if (hum < 10.0f) sHum = "0" + sHum;
  
  display.setFont(ArialMT_Plain_24);
  int wHum = display.getStringWidth(sHum);
  int wTemp = display.getStringWidth(sTemp);
  display.drawString(2, 9, sTemp);
  display.drawString(125-wHum, 9, sHum);

  display.setFont(ArialMT_Plain_10);
  String sHumText = "% hum";
  String sTempText = "°C temp";
  int wHumText = display.getStringWidth(sHumText);
  int wTempText = display.getStringWidth(sTempText);
  display.drawString(4, 0, sTempText);
  display.drawString(123-wHumText, 0, sHumText);

  if (hum>warnThreshold)
  {
    display.setFont(ArialMT_Plain_10);
    String sWarn = "WARN!";
    int wWarn = display.getStringWidth(sWarn);
    int pWarn = (127-wTempText-wHumText)/2 + wTempText - wWarn/2;
    display.drawString(pWarn, 0, sWarn);
    display.invertDisplay();
  }
  else
  {
    display.normalDisplay();
  }
  
  display.display();
  printDebug("Display data - done.");
}

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
  ThingsBoard tb(wifiClient);
  printDebug("Connecting to ThingsBoard...");
  if (debug) digitalWrite(LED_ST_1, HIGH);

  if (tb.connect(configuration["thingsboardServer"].c_str(), configuration["thingsboardToken"].c_str()))
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
    if (debug) digitalWrite(LED_ST_1, LOW);
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

void startConnectWiFi()
{
  printDebug("Connecting to WiFi...");
  if (debug) digitalWrite(LED_ST_0, HIGH);

  // attempt to connect to WiFi network
  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  WiFi.config(staticIP, gateway, subnet);
  WiFi.begin(configuration["wifiSsid"].c_str(), configuration["wifiPassword"].c_str());
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

      printError("WiFi connect failed! Retry...");
      --remainigRetries;
      timeoutEnd = millis() + TIMEOUT;
    }

    if (millis() > timeoutEnd)
    {
      printError("WiFi timeout with WiFi.status(" + String(WiFi.status()) + ")!");
      return false;
    }

    if (isForceConfig()) return false;
    
    delay(10);
  }

  if (debug) Serial.setDebugOutput(false);
  printDebug("Connecting to WiFi - done.");
  return true;
}

void disconnectWiFi()
{
  // printDebug("Disconnecting from WiFi.");
  // WiFi.disconnect(true);
  // delay(1);
  digitalWrite(LED_ST_0, LOW);
}

void printError(const String & s)
{
  unsigned long time = millis();
  int milliseconds = time % 1000;
  int seconds = (time / 1000) % 60;
  int minutes = (time / 1000 / 60) % 60;
  int hours = time / 1000 / 60 / 60;
  char buf[32];
  sprintf(buf,"%4d:%02d:%02d.%03d ", hours, minutes, seconds, milliseconds);
  Serial.print(buf);
  Serial.println(s);
}

void printDebug(const String & s)
{
  if (debug) printError(s);
}
