#include <Arduino.h>
#include <LedController.hpp>
#include <NewPing.h>
#include <LedMatrixPatterns.h>
#include "../include/version.h"
#include <wsData.h>
#include <helpers.h>
#include "credentials.h"

#define SONAR_NUM 2
#define DISPLAY_INTENSITY 0 // Set the brightness (0 to 15) [0] 8
#define MIN_DISTANCE 0      // [>13] 20
#define MAX_DISTANCE 200    // [<217] 200
#define MAX_TIMEOUT 25000   // Turn off 8x8 in ms 25000
#define ITERATIONS 5        // [10] 5
#define MAX_CHANGES_PER_MINUTE 10
#define MILLIS_PER_MINUTE 60000

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
NewPing sonar[SONAR_NUM] = {
    NewPing(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE),
    NewPing(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE)};

const unsigned long PING_DELAY = 50; // 50 [100] Better with 150ms without Serial
unsigned long lastMillisDisplayTimeout = 0;
boolean timeout = false;

unsigned long lastGarageChangeTime = 0;
unsigned int garageChanges = 0;
bool isGarageFull = false;

struct Cars
{
  int previousDistance[SONAR_NUM];
  bool present[SONAR_NUM];
};

Cars myCars;

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

void initializeCars()
{
  for (int i = 0; i < SONAR_NUM; i++)
  {
    myCars.previousDistance[i] = 0;
    myCars.present[i] = false;
  }
}

void checkCarPresence(int sensorNum, NewPing &sonar, bool &isCarPresent, int &prevDistance)
{
#ifdef ITERATIONS
  unsigned int distance = 0;
  unsigned int cm[ITERATIONS];
  int zeroCount = 0;
  int newDistance = 0;
  for (uint8_t i = 0; i < ITERATIONS; i++)
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
#ifdef DEBUG
  Serial.print(F("> Sensor"));
  Serial.print(sensorNum);
  Serial.print(F(": zeroCount = "));
  Serial.println(zeroCount);
#endif
  if (zeroCount >= (ITERATIONS - 1))
  {
    distance = 0;
  }
  else
  {
    distance = newDistance;
  }
#else
  delay(PING_DELAY);
  unsigned int distance = sonar.ping_cm();
#endif

  myData.distances[sensorNum] = distance;

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
      myData.cars[sensorNum] = isCarPresent;
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
      myData.cars[sensorNum] = isCarPresent;
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
  delay(1000);
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

void setup()
{
  initSerial();
  printBootMsg();
  initDisplay(lc, DISPLAY_INTENSITY);
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
  // Initalize cars
  initializeCars();
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
  for (uint8_t i = 0; i < SONAR_NUM; i++)
  {
    checkCarPresence(i, sonar[i], myCars.present[i], myCars.previousDistance[i]);
  }

  // Check if cars are present
  if (myCars.present[0] && myCars.present[1])
  {
    if (!isGarageFull)
    {
      if ((millis() - lastGarageChangeTime) >= MILLIS_PER_MINUTE / MAX_CHANGES_PER_MINUTE)
      {
        isGarageFull = true;
        lastGarageChangeTime = millis();
        garageChanges++;
#ifdef VERBOSE
        Serial.print(F("> Garage: "));
        Serial.println(isGarageFull);
#endif
        writeMatrixInv(lc, smile);
        lastMillisDisplayTimeout = millis();
        timeout = false;
        myData.garageFull = isGarageFull;
        notifyClients();
        mqttClient.publish((String(mqtt_topic) + "/isFull").c_str(), boolToString(isGarageFull), true);
        // Reset garageChanges counter after reaching the limit
        if (garageChanges >= MAX_CHANGES_PER_MINUTE)
        {
          garageChanges = 0;
          delay(MILLIS_PER_MINUTE / MAX_CHANGES_PER_MINUTE);
        }
      }
    }
  }
  else
  {
    if (isGarageFull)
    {
      if ((millis() - lastGarageChangeTime) >= MILLIS_PER_MINUTE / MAX_CHANGES_PER_MINUTE)
      {
        isGarageFull = false;
        lastGarageChangeTime = millis();
        garageChanges++;
#ifdef VERBOSE
        Serial.print(F("> Garage: "));
        Serial.println(isGarageFull);
#endif
        writeMatrixInv(lc, null); // [null]
        timeout = true;
        myData.garageFull = isGarageFull;
        notifyClients();
        mqttClient.publish((String(mqtt_topic) + "/isFull").c_str(), boolToString(isGarageFull), true);
        // Reset garageChanges counter after reaching the limit
        if (garageChanges >= MAX_CHANGES_PER_MINUTE)
        {
          garageChanges = 0;
          delay(MILLIS_PER_MINUTE / MAX_CHANGES_PER_MINUTE);
        }
      }
    }
#ifdef DEBUG
    else
    {
      Serial.print(F("> Garage: "));
      Serial.println(isGarageFull);
      writeMatrixInv(lc, error);
    }
#endif
  }

  // Check for timeout
  if (isGarageFull && (millis() - lastMillisDisplayTimeout) > MAX_TIMEOUT)
  {
    if (!timeout)
    {
#ifdef VERBOSE
      Serial.println(F("> 8x8: OFF TIMEOUT"));
#endif
      writeMatrixInv(lc, null); // [null]
      lastMillisDisplayTimeout = 0;
      timeout = true;
    }
  }
}
