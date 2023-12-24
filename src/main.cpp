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
#define MAX_DISTANCE 250    // Change detection distance in cm [350]
#define MAX_TIMEOUT 30000   // Turn off 8x8 in ms
#define ITERATIONS 8

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

LedController lc = LedController(PIN_DATA, PIN_CLK, PIN_CS, 1);
NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);

const unsigned long PING_DELAY = 50; // [100] Better with 150ms without Serial
unsigned long lastMillisDisplayTimeout = 0;

boolean timeout = false;

bool isCarPresent1 = false;
bool isCarPresent2 = false;
bool areCarsPresent = false;

int previousDistance1 = 0;
int previousDistance2 = 0;

// ESP
#if defined(ESP8266)
String hostname = "esp8266-";
#endif
#if defined(ESP32)
String hostname = "esp32-";
#endif
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

void checkCarPresence(int sensorNum, NewPing &sonar, bool &isCarPresent, int &prevDistance)
{
  delay(PING_DELAY);
  unsigned int distance = sonar.ping_cm();

  unsigned int cm[ITERATIONS];
  int zeroCount = 0;
  int newDistance = 0;
  for (uint8_t i = 1; i < ITERATIONS; i++)
  {
    delay(PING_DELAY);
    distance = sonar.ping_cm();
    cm[i] = distance;
    if (cm[i] == 0)
    {
      zeroCount++;
    }
    else
    {
      newDistance = distance;
    }
  }
  if (zeroCount >= (ITERATIONS-1) / 2)
  {
    distance = 0;
  } else {
    distance = newDistance;
  }

  if ((prevDistance > 0 && distance == 0) || (prevDistance == 0 && distance > 0))
  {
#ifdef VERBOSE
    Serial.print(F("> Sensor"));
    Serial.print(sensorNum);
    Serial.print(F(": "));
    Serial.println(F("zero correction"));
#endif
  }
  else
  {
    if (sensorNum == 1)
    {
      myData.distance1 = distance;
    }
    else
    {
      myData.distance2 = distance;
    }
#ifdef DEBUG
    Serial.print(F("> Sensor"));
    Serial.print(sensorNum);
    Serial.print(F(": "));
    Serial.print(distance);
    Serial.println(F("cm"));
#endif
    if (distance > MIN_DISTANCE && distance < MAX_DISTANCE)
    {
      if (!isCarPresent)
      {
#ifdef VERBOSE
        Serial.print(F("> Car"));
        Serial.print(sensorNum);
        Serial.println(F(": True"));
#endif
        isCarPresent = true;
        if (sensorNum == 1)
        {
          myData.car1 = isCarPresent;
        }
        else
        {
          myData.car2 = isCarPresent;
        }
      }
    }
    else
    {
      if (isCarPresent)
      {
#ifdef VERBOSE
        Serial.print(F("> Car"));
        Serial.print(sensorNum);
        Serial.println(F(": False"));
#endif
        isCarPresent = false;
        if (sensorNum == 1)
        {
          myData.car1 = isCarPresent;
        }
        else
        {
          myData.car2 = isCarPresent;
        }
      }
    }
  }
  prevDistance = distance;
  notifyClients();
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
#if defined(ESP8266) || defined(ESP32)
  Serial.print(F("> Node ID: "));
  Serial.println(getUniqueID());
  hostname += getUniqueID();
#endif
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
  loadingAnimation(lc);
  lc.setIntensity(DISPLAY_INTENSITY);
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

  checkCarPresence(1, sonar1, isCarPresent1, previousDistance1);
  checkCarPresence(2, sonar2, isCarPresent2, previousDistance2);

  // Check if cars are present
  if (isCarPresent1 && isCarPresent2)
  {
    if (!areCarsPresent)
    {
#ifdef VERBOSE
      Serial.println(F("> Cars: True"));
#endif
      writeMatrixInv(lc, smile);
      areCarsPresent = true;
      lastMillisDisplayTimeout = millis();
      timeout = false;
      myData.cars = areCarsPresent;
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
      writeMatrixInv(lc, arrow); // [null]
      areCarsPresent = false;
      timeout = true;
      myData.cars = areCarsPresent;
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
      writeMatrixInv(lc, ex); // [null]
      lastMillisDisplayTimeout = 0;
      timeout = true;
    }
  }
}
