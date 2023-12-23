#include <Arduino.h>
#include <LedController.hpp>
#include <NewPing.h>
#include "LedMatrixPatterns.h"
#include "version.h"
#include "wsData.h"
#include "helpers.h"
#include "credentials.h"

#define DISPLAY_INTENSITY 0 // Set the brightness (0 to 15) [0] 8
#define MIN_DISTANCE 0      //
#define MAX_DISTANCE 200    // Change detection distance in cm [350]
#define MAX_TIMEOUT 30000   // Turn off 8x8 in ms

#if defined(ESP8266) || defined(ESP32)
// MAX7218
#define PIN_CLK D5
#define PIN_CS D8
#define PIN_DATA D7
// HC-SR04 #1
#define ECHO_PIN_1 D1
#define TRIGGER_PIN_1 D2
// HC-SR04 #2
#define ECHO_PIN_2 D3
#define TRIGGER_PIN_2 D4
#elif defined(ARDUINO_AVR_UNO)
// MAX7218
#define PIN_CLK 13
#define PIN_CS 10
#define PIN_DATA 11
// HC-SR04 #1
#define ECHO_PIN_1 2
#define TRIGGER_PIN_1 3
// HC-SR04 #2
#define ECHO_PIN_2 4
#define TRIGGER_PIN_2 5
#elif defined(ARDUINO_AVR_PROMICRO)
// MAX7218
#define PIN_CLK 15  // SCLK
#define PIN_CS 10   //
#define PIN_DATA 16 // MOSI
// HC-SR04 #1
#define ECHO_PIN_1 2
#define TRIGGER_PIN_1 3
// HC-SR04 #2
#define ECHO_PIN_2 4
#define TRIGGER_PIN_2 5
#else
Serial.println("Unknown board type");
#endif

LedController lc = LedController(PIN_DATA, PIN_CLK, PIN_CS, 1);
NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);

const unsigned long PING_DELAY = 150; // [100] Better with 150ms without Serial
unsigned long lastMillisDisplayTimeout = 0;

boolean timeout = false;

bool isCarPresent1 = false;
bool isCarPresent2 = false;
bool areCarsPresent = false;

#if defined(ESP8266)
String hostname = "esp8266-";
#endif
#if defined(ESP32)
String hostname = "esp32-";
#endif

// ESP
// WiFi & MQTT
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
wsData myData;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 3600);

long mqttLastReconnectAttempt = 0;
int wsDataSize = 0;
uint8_t connectedClients = 0;

unsigned long previousMinute = 0;

// supplementary functions
#ifdef VERBOSE
// one minute mark
#define MARK
unsigned long lastMillisMark = 0L;
uint32_t countMsg = 0;
#endif

void checkCarPresence(const char *sensorName, NewPing &sonar, bool &isCarPresent)
{
  delay(PING_DELAY);

  unsigned int distance = sonar.ping_cm();

#ifdef DEBUG
  Serial.print(F("> "));
  Serial.print(sensorName);
  Serial.print(F(": "));
  Serial.print(distance);
  Serial.println(F("cm"));
#endif

  if (distance > MIN_DISTANCE && distance < MAX_DISTANCE)
  {
    if (!isCarPresent)
    {
#ifdef VERBOSE
      Serial.print(F("> Car "));
      Serial.print(sensorName);
      Serial.println(F(": True"));
#endif
      isCarPresent = true;
    }
  }
  else
  {
    if (isCarPresent)
    {
#ifdef VERBOSE
      Serial.print(F("> Car "));
      Serial.print(sensorName);
      Serial.println(F(": False"));
#endif
      isCarPresent = false;
    }
  }
}

void initSerial()
{
  Serial.begin(115200);
  delay(10);
}

void printBootMsg()
{
#ifdef DEBUG
  delay(5000);
#endif
  // Start Boot
  Serial.println(F("> "));
  Serial.println(F("> "));
  Serial.print(F("> Booting... Compiled: "));
  Serial.println(VERSION);
#ifdef VERBOSE
  Serial.print(("> Mode: "));
  Serial.print(F("VERBOSE "));
#ifdef DEBUG
  Serial.print(F("DEBUG"));
#endif
  Serial.println();
#endif
}

void initDisplay()
{
#ifdef VERBOSE
  lc.clearMatrix();
  lc.setIntensity(DISPLAY_INTENSITY);
  loadingAnimation(lc);
  lc.clearMatrix();
#endif
}

void setup()
{
  initSerial();
  initDisplay();
  printBootMsg();
  initFS();
  checkWiFi();
  mqttClient.setServer(mqtt_server, mqtt_port);
#ifdef MQTT_SUBSCRIBE
  mqttClient.setCallback(onMqttMessage);
#endif
  if (WiFi.status() == WL_CONNECTED)
  {
    initMDNS();
    connectToMqtt();
    timeClient.begin();
    timeClient.update();
    myData.boottime = timeClient.getEpochTime();
  }
  // Initalize websocket
  initWebSocket();
}

void loop()
{
  ws.cleanupClients();
#ifdef REQUIRES_INTERNET
  checkWiFi();
#endif
  checkMqtt();
#ifdef MARK
  printMARK();
#endif

  checkCarPresence("Sensor1", sonar1, isCarPresent1);
  checkCarPresence("Sensor2", sonar2, isCarPresent2);

  // Check if cars are present
  if (isCarPresent1 && isCarPresent2)
  {
    if (!areCarsPresent)
    {
#ifdef VERBOSE
      Serial.println(F("> Cars: True"));
#endif
      writeMatrix(lc, smile);
      areCarsPresent = true;
      lastMillisDisplayTimeout = millis();
      timeout = false;
      myData.car = areCarsPresent;
      notifyClients();
    }
  }
  else
  {
    if (areCarsPresent)
    {
#ifdef VERBOSE
      Serial.println(F("> Cars: False"));
#endif
      writeMatrix(lc, null);
      areCarsPresent = false;
      timeout = true;
      myData.car = areCarsPresent;
      notifyClients();
    }
  }

  // Check for timeout
  if (areCarsPresent && (millis() - lastMillisDisplayTimeout) > MAX_TIMEOUT)
  {
    if (!timeout)
    {
#ifdef VERBOSE
      Serial.println(F("> 8x8: OFF TIMEOUT"));
#endif
      writeMatrix(lc, null);
      lastMillisDisplayTimeout = 0;
      timeout = true;
    }
  }
}
