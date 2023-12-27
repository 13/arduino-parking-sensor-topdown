// wsData.h

#ifndef WSDATA_H
#define WSDATA_H

#include <Arduino.h>
#include <ArduinoJson.h>

const int MAX_CARS = 2;

struct wsData
{
    int uptime;
    int rssi;
    int memfree;
    int memfrag;
    String ssid;
    String ip;
    String mac;
    String cpu;
    String hostname;
    String desc;
    String resetreason;
    String version;
    time_t boottime;
    time_t timestamp;
    // parking sensor
    boolean cars[MAX_CARS];
    int distances[MAX_CARS];
    boolean garageFull;

    String toJson()
    {
        // Create a JSON document
        DynamicJsonDocument doc(2048);

        // Add fields to the document
        doc["uptime"] = uptime;
        doc["rssi"] = rssi;
        doc["memfree"] = memfree;
        doc["memfrag"] = memfrag;
        doc["ssid"] = ssid;
        doc["ip"] = ip;
        doc["mac"] = mac;
        doc["cpu"] = cpu;
        doc["hostname"] = hostname;
        doc["desc"] = desc;
        doc["resetreason"] = resetreason;
        doc["version"] = version;
        doc["boottime"] = boottime;
        doc["timestamp"] = timestamp;
        // parking sensor
        doc["garageFull"] = garageFull;

        // Add the packets array to the document
        JsonArray carsArray = doc.createNestedArray("cars");
        for (int i = 0; i < MAX_CARS; ++i)
        {
            carsArray.add(cars[i]);
        }
        JsonArray distancesArray = doc.createNestedArray("distances");
        for (int i = 0; i < MAX_CARS; ++i)
        {
            distancesArray.add(distances[i]);
        }

        // Serialize the document to a JSON string
        String jsonString;
        serializeJson(doc, jsonString);
        // doc.~BasicJsonDocument(); // destroy

        return jsonString;
    }
};

#endif // WSDATA_H
