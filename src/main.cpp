#include <Arduino.h>
#include <LedController.hpp>
#include <NewPing.h>
#include "LedMatrixPatterns.h"
#include "version.h"

#define VERBOSE
// #define DEBUG

#define DISPLAY_INTENSITY 0 // Set the brightness (0 to 15) [8] 0
#define MIN_DISTANCE 0      //
#define MAX_DISTANCE 30     // Change detection distance in cm [350]
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

const unsigned long PING_DELAY = 100;
unsigned long lastMillis = 0;

boolean timeout = false;

bool isCarPresent1 = false;
bool isCarPresent2 = false;
bool areCarsPresent = false;

void setup()
{
  Serial.begin(115200);
  delay(10);
#ifdef VERBOSE
  lc.clearMatrix();
  lc.setIntensity(DISPLAY_INTENSITY);
  loadingAnimation(lc);
  lc.clearMatrix();
#endif
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
  // max7219
  lc.clearMatrix();
  lc.setIntensity(DISPLAY_INTENSITY);
}

void loop()
{
  // Read distance from Sonar1
  delay(PING_DELAY);

  unsigned int distance1 = sonar1.ping_cm();
#ifdef DEBUG
  Serial.print(F("> Sensor1: "));
  Serial.print(distance1);
  Serial.println(F("cm"));
#endif

  // Check if car is present
  if (distance1 > MIN_DISTANCE && distance1 < MAX_DISTANCE)
  {
    if (!isCarPresent1)
    {
#ifdef VERBOSE
      Serial.println(F("> Car1: True"));
#endif
      isCarPresent1 = true;
    }
  }
  else
  {
    if (isCarPresent1)
    {
#ifdef VERBOSE
      Serial.println(F("> Car1: False"));
#endif
      isCarPresent1 = false;
    }
  }

  // Read distance from Sonar2
  delay(PING_DELAY);

  unsigned int distance2 = sonar2.ping_cm();
#ifdef DEBUG
  Serial.print(F("> Sensor2: "));
  Serial.print(distance2);
  Serial.println(F("cm"));
#endif

  // Check if car is present
  if (distance2 > MIN_DISTANCE && distance2 < MAX_DISTANCE)
  {
    if (!isCarPresent2)
    {
#ifdef VERBOSE
      Serial.println(F("> Car2: True"));
#endif
      isCarPresent2 = true;
    }
  }
  else
  {
    if (isCarPresent2)
    {
#ifdef VERBOSE
      Serial.println(F("> Car2: False"));
#endif
      isCarPresent2 = false;
    }
  }

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
      lastMillis = millis();
      timeout = false;
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
    }
  }

  // Check for timeout
  if (areCarsPresent && (millis() - lastMillis) > MAX_TIMEOUT)
  {
    if (!timeout)
    {
#ifdef VERBOSE
      Serial.println(F("> 8x8: OFF TIMEOUT"));
#endif
      writeMatrix(lc, null);
      lastMillis = 0;
      timeout = true;
    }
  }
}
