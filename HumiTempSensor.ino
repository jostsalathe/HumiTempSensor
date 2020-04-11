#include <BME280I2C.h>
#include <Wire.h>
#include <SSD1306Wire.h>
#include <ESP8266WiFi.h>
#include <FS.h>
#include <ThingsBoard.h>

String wifiSsid;
String wifiPassword;
IPAddress staticIP;
IPAddress gateway;
IPAddress subnet;

String thingsboardServer;
String token;

long intervalMs;


#define DEBUG_EN 14
#define LED_ST_0 4
#define LED_ST_1 5
#define GPIO12 12
#define GPIO13 13

// I2C

#define SDA 2
#define SCL 0

WiFiClient wifiClient;

// Initialize SSD1306 OLED.
SSD1306Wire display(0x3c, SDA, SCL, GEOMETRY_128_32);

// Initialize BME280I2C sensor.
BME280I2C::Settings bmeSettings(
  BME280I2C::OSR_X1, // temp oversampling
  BME280I2C::OSR_X1, // humidity oversampling
  BME280I2C::OSR_X1, // pressure oversampling
  BME280I2C::Mode_Forced,
  BME280I2C::StandbyTime_1000ms,
  BME280I2C::Filter_Off,
  BME280I2C::SpiEnable_False,
  0x76 // I2C address. (0x76 is default, 0x77 alternative)
);
BME280I2C bme(bmeSettings);

bool debug = true;

unsigned long lastSend;


void setup()
{
  pinMode(DEBUG_EN, INPUT_PULLUP);
  // internally pulled high - solder bridge open
  if (digitalRead(DEBUG_EN)) debug = false;
  else debug = true;
  pinMode(LED_ST_0, OUTPUT);
  pinMode(LED_ST_1, OUTPUT);
  digitalWrite(LED_ST_0, LOW);
  digitalWrite(LED_ST_1, LOW);
  
  Serial.begin(74880);
  Wire.begin(SDA, SCL);

  if (readConfig()) getAndReportSensorDataThenSleep();
  else Serial.println("handling faulty configuration");
}

void loop()
{
  digitalWrite(LED_ST_1, HIGH);
  delay(50);
  digitalWrite(LED_ST_0, HIGH);
  delay(200);
  digitalWrite(LED_ST_1, LOW);
  delay(50);
  digitalWrite(LED_ST_0, LOW);
  delay(200);
}

bool readConfig()
{
  if (debug) Serial.println("reading configuration files from SPIFFS...");

  // do stuff
  SPIFFS.begin();

  bool success = true;
  if (!readFirstLineFromFile(wifiSsid, "/ssid.txt"))            success = false;
  if (!readFirstLineFromFile(wifiPassword, "/pwd.txt"))         success = false;
  if (!readFirstIpFromFile(staticIP, "/ip.txt"))                success = false;
  if (!readFirstIpFromFile(gateway, "/gateway.txt"))            success = false;
  if (!readFirstIpFromFile(subnet, "/subnet.txt"))              success = false;

  if (!readFirstLineFromFile(thingsboardServer, "/server.txt")) success = false;
  if (!readFirstLineFromFile(token, "/token.txt"))              success = false;

  if (!readFirstPositiveIntegerFromFile(intervalMs, "/intervalMs.txt"))    success = false;
  if (intervalMs < 1)
  {
    Serial.println("ERROR: measurement interval is " + String(intervalMs) + " but must be >0");
    success = false;
  }
  
  if (debug) Serial.println("reading configuration files from SPIFFS - done.");
  return success;
}

bool openFile(File & f, String fileName)
{
  if (debug) Serial.println("reading file \"" + fileName + "\"");
  f = SPIFFS.open(fileName, "r");
  if (!f) {
    Serial.println("ERROR: couldn't open file \"" + fileName + "\"");
    f.close();
    return false;
  }
  return true;
}

bool readFirstLineFromFile(String & ret, String fileName)
{
  File f;
  if (!openFile(f, fileName)) return false;
  ret = f.readString();
  ret = ret.substring(0, ret.indexOf('\r'));
  ret = ret.substring(0, ret.indexOf('\n'));
  f.close();
  return true;
}

bool readFirstPositiveIntegerFromFile(long & ret, String fileName)
{
  String sNumber;
  if (!readFirstLineFromFile(sNumber, fileName)) return false;
  else if ((ret = sNumber.toInt()) <= 0)
  {
    Serial.println("ERROR: couldn't convert \"" + sNumber + "\" to positive integer");
    return false;
  }
  return true;
}

bool readFirstIpFromFile(IPAddress & ret, String fileName)
{
  String sAddr;
  if (!readFirstLineFromFile(sAddr, fileName)) return false;
  else if (!ret.fromString(sAddr))
  {
    Serial.println("ERROR: couldn't convert \"" + sAddr + "\" to IP address");
    return false;
  }
  return true;
}

void getAndReportSensorDataThenSleep()
{
  startCconnectWiFi();  
  
  bme.begin();
  
  if (debug) Serial.println("Acquire new sensor data...");
  // pressure in hPa
  float pressure = NAN;
  // temperature in Celsius
  float temperature = NAN;
  // humidity in percent
  float humidity = NAN;

  // read sensor
  bme.read(pressure, temperature, humidity, BME280I2C::TempUnit_Celsius, BME280I2C::PresUnit_hPa);
  delay(1000);
  bme.read(pressure, temperature, humidity, BME280I2C::TempUnit_Celsius, BME280I2C::PresUnit_hPa);
  if (debug) Serial.println("Measured: pres=" + String(pressure,2) + "hPa, temp=" + String(temperature,2) + "°C, hum=" + String(humidity,2) + "%");

  displayMeasurements(humidity, temperature, pressure);
  reportMeasurements(humidity, temperature, pressure);
  
  long sleepMs = intervalMs - millis();
  if (sleepMs < 1) sleepMs = 1;
  if (debug) Serial.println("Entering deep sleep for " + String(sleepMs) + "ms...");
  ESP.deepSleep(static_cast<uint64_t>(sleepMs) * 1000);
}

void displayMeasurements(float hum, float temp, float pres)
{
  display.init();
  display.flipScreenVertically();
  display.setBrightness(1);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.clear();

  display.drawString(0, 0, "Humi:");
  display.drawString(0, 11, "Temp:");
  display.drawString(0, 22, "Pres:");
  
  if (!isnan(hum)) display.drawString(40, 0, String(hum,2)+" %");
  if (!isnan(temp)) display.drawString(40, 11, String(temp,2)+" °C");
  if (!isnan(pres)) display.drawString(40, 22, String(pres,0)+" hPa");

  if (hum>60.0f) display.invertDisplay();
  else display.normalDisplay();
  
  display.display();
}

void reportMeasurements(float hum, float temp, float pres)
{
  if (debug) Serial.println("Transmit to server...");
  
  if (isnan(hum) || isnan(temp) || isnan(pres))
  {
    if (debug) Serial.println("ERROR: Sensor read failed, faulty data not transmitted!");
    return;
  }

  // wait for WiFi connection
  waitConnectWiFi();

  // connecting to ThingsBoard
  ThingsBoard tb(wifiClient);
  if (debug)
  {
    Serial.println("Connecting to ThingsBoard...");
    digitalWrite(LED_ST_1, HIGH);
  }
  if (tb.connect(thingsboardServer.c_str(), token.c_str()))
  {
    if (debug) Serial.println("Sending data to ThingsBoard...");
    // sending data to ThingsBoard
    const int data_items = 3;
    Telemetry data[data_items] =
    {
      {"temperature", temp},
      {"humidity",    hum},
      {"pressure",    pres}
    };
    tb.sendTelemetry(data, data_items);
  }
  else if (debug)
  {
    Serial.println("Connecting to ThingsBoard failed!");
    digitalWrite(LED_ST_1, LOW);
  }

  // disconnecting from ThingsBoard
  if (debug) Serial.println("Disconnecting from ThingsBoard...");
  tb.disconnect();
  digitalWrite(LED_ST_1, LOW);
  
  // disconnecting from WiFi
  disconnectWiFi();
  
  if (debug) Serial.println("Transmit to server - done.");
}

bool startCconnectWiFi()
{
  if (debug)
  {
    digitalWrite(LED_ST_0, HIGH);
    Serial.println("Connecting to WiFi...");
  }

  // attempt to connect to WiFi network
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.config(staticIP, gateway, subnet);
  WiFi.begin(wifiSsid, wifiPassword);
}

bool waitConnectWiFi()
{
  if (debug) Serial.println("Wait for WiFi...");
  while (WiFi.status() != WL_CONNECTED) delay(10);
  if (debug)
  {
    Serial.setDebugOutput(false);
    Serial.println("Connecting to WiFi - done.");
  }
}

void disconnectWiFi()
{
  if (debug) Serial.println("Disconnecting from WiFi.");
  WiFi.disconnect(true);
  delay(1);
  digitalWrite(LED_ST_0, LOW);
}
