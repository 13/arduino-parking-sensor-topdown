#include <Arduino.h>
#include <LedController.hpp>
#include <NewPing.h>
#include <LedMatrixPatterns.h>
#include "../include/version.h"
#include <wsData.h>
#include <ArduinoEspHelper.h>
#include "credentials.h"

#define SONAR_NUM 2         // Num of sensors [2]
#define DISPLAY_INTENSITY 0 // Set the brightness (0-15) [0]
#define MIN_DISTANCE 0      // [0]
#define MAX_DISTANCE 190    // 200 [190] 170
#define MAX_TIMEOUT 25000   // Turn off 8x8 [25000]
#define PING_DELAY 80       // 50 [100] [150](w/o Serial)
#define ITERATIONS 4        // [5] 8

#if defined(ESP8266)
// MAX7218
#define PIN_CLK D5
#define PIN_CS D8
#define PIN_DATA D7
// HC-SR04 #1
#define ECHO_PIN_1 D1
#define TRIGGER_PIN_1 D2
// HC-SR04 #2
#define ECHO_PIN_2 D6
#define TRIGGER_PIN_2 D3
#endif
#if defined(ESP32)
// MAX7218
#define PIN_CLK 14
#define PIN_CS 12
#define PIN_DATA 13
// HC-SR04 #1
#define ECHO_PIN_1 23    // 23
#define TRIGGER_PIN_1 22 // 22
// HC-SR04 #2
#define ECHO_PIN_2 19    // 19
#define TRIGGER_PIN_2 18 // 18
#endif

LedController lc = LedController(PIN_DATA, PIN_CLK, PIN_CS, 1);

NewPing sonar[SONAR_NUM] = {
    NewPing(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE),
    NewPing(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE)};

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
#if defined(ITERATIONS) && ITERATIONS > 1
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
      isGarageFull = true;

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
      /*if ((millis() - lastGarageChangeTime) > 3000 || lastGarageChangeTime == 0)
      {
        // If there is no change for 5 seconds, publish to MQTT
        mqttClient.publish((String(mqtt_topic) + "/isFull").c_str(), boolToString(isGarageFull), true);
        lastGarageChangeTime = millis(); // Reset the timer
      }*/
    }
  }
  else
  {
    if (isGarageFull)
    {
      isGarageFull = false;

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
      /*if ((millis() - lastGarageChangeTime) > 3000 || lastGarageChangeTime == 0)
      {
        mqttClient.publish((String(mqtt_topic) + "/isFull").c_str(), boolToString(isGarageFull), true);
        lastGarageChangeTime = millis(); // Reset the timer
      }*/
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
