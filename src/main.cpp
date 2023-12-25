#include <Arduino.h>
#include <LedController.hpp>
#include <NewPing.h>
#include "LedMatrixPatterns.h"
#include "version.h"

#define VERBOSE
#define DEBUG

#define DISPLAY_INTENSITY 0 // Set the brightness (0 to 15) [0] 8
#define MIN_DISTANCE 0      //
#define MAX_DISTANCE 190    // [216]
#define MAX_TIMEOUT 25000   // Turn off 8x8 in ms
#define ITERATIONS 5        // [10]

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

int previousDistance1 = 0;
int previousDistance2 = 0;

void checkCarPresence(int sensorNum, NewPing &sonar, bool &isCarPresent, int &prevDistance)
{
#ifdef ITERATIONS
  unsigned int distance = 0;
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
    }
  }
  prevDistance = distance;
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

void setup()
{
  initSerial();
  printBootMsg();
  initDisplay(lc, DISPLAY_INTENSITY);
}

void loop()
{
  checkCarPresence(1, sonar1, isCarPresent1, previousDistance1);
  checkCarPresence(2, sonar2, isCarPresent2, previousDistance2);
  // Check if cars are present
  if (isCarPresent1 && isCarPresent2)
  {
    if (!areCarsPresent)
    {

      areCarsPresent = true;
#ifdef VERBOSE
      Serial.print(F("> Cars: "));
      Serial.println(areCarsPresent);
#endif
      writeMatrixInv(lc, smile);
      lastMillisDisplayTimeout = millis();
      timeout = false;
    }
  }
  else
  {
    if (areCarsPresent)
    {
      areCarsPresent = false;
#ifdef VERBOSE
      Serial.print(F("> Cars: "));
      Serial.println(areCarsPresent);
#endif
      writeMatrixInv(lc, arrow); // [null]
      timeout = true;
    }
#ifdef DEBUG
    else
    {
      Serial.print(F("> Cars: "));
      Serial.println(areCarsPresent);
      writeMatrixInv(lc, num5);
    }
#endif
  }

  // Check for timeout
  if (areCarsPresent && (millis() - lastMillisDisplayTimeout) > MAX_TIMEOUT)
  {
    if (!timeout)
    {
#ifdef VERBOSE
      Serial.println(F("> 8x8: OFF TIMEOUT"));
#endif
      writeMatrixInv(lc, timedout); // [null]
      lastMillisDisplayTimeout = 0;
      timeout = true;
    }
  }
}