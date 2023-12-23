#include <Arduino.h>

#define VERBOSE
// #define DEBUG

// #define REQUIRES_INTERNET
// #define MQTT_SUBSCRIBE

/* WiFi */
const char *wifi_ssid = "network";
const char *wifi_pass = "";

const char *desc = "arduino parking assistant";

/* MQTT server credentials */
const char *mqtt_user = "";
const char *mqtt_pass = "";
const char *mqtt_server = "192.168.22.5";
const char *mqtt_topic = "muh/sensors";
const char *mqtt_topic_lwt = "muh/esp";
uint16_t mqtt_port = 1883;