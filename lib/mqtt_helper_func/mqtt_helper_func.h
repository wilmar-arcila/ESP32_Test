//
//    FILE: mqtt_helper_func.h
//  AUTHOR: Wilmar Arcila
// VERSION: 0.0.1
//    DATE: 2022-06-09
// PURPOSE: Helper functions to easy the connection to a broker using mqtt
//
//     See: https://randomnerdtutorials.com/cloud-mqtt-mosquitto-broker-access-anywhere-digital-ocean/
//          https://randomnerdtutorials.com/epoch-unix-time-esp32-arduino/
//
//  HISTORY:
//  0.0.1   2022-06-09  initial version

#pragma once

#ifndef __mqtt_helper_func__
#define __mqtt_helper_func__

#include <wifi_helper_func.h>
#include "/home/wilmar/IoT/ESP32_Test/lib/mqtt_helper_func/mqtt_config.h"

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>

struct singleVariable{
  float value;
  String varContext;
};

#define CAPACITY JSON_OBJECT_SIZE(MQTT_TOTAL_VARS*5+8)


class mqttComm
{
  public:

    mqttComm();           // Constructor
    void mqttInit(void);  // Inicializaciones
    void printMessage(void);
    bool setVariable(uint8_t, float, String);
    void publish(void);
    void mqttProcessMsg(char*);
    singleVariable getLastMessage(void);
    uint64_t getTxTimestamp(void) {return _txTimestamp;};
    uint64_t getRxTimestamp(void) {return _rxTimestamp;};
  private:
    bool _mqttConnected;
    String _sensorId;                 // Identificador único del sensor
    String _status;                   // Estado general del sensor
    uint8_t _priority;                // Prioridad del mensaje -> 0: urgente, 1: asap, 2: normal
    uint64_t _rxTimestamp;
    uint64_t _txTimestamp;
    const char* _ntpServer;            // NTP server to request epoch time
    uint64_t _epochTimems;        // Variable to save current epoch time

    singleVariable _receivedVariable;
    singleVariable _MQTT_SENSOR1_VAR;
    singleVariable _MQTT_SENSOR2_VAR;
    singleVariable _MQTT_SENSOR3_VAR;
    singleVariable* _varPointers[MQTT_TOTAL_VARS];

    StaticJsonDocument<CAPACITY> _filter;  // The filter: it contains "true" for each value we want to keep

    void mqttConnectToBroker(void);
    StaticJsonDocument<CAPACITY> mqttCreateMessage(void);
    unsigned long getTime(void);
};

void onMqttConnect(bool);
void onMqttDisconnect(AsyncMqttClientDisconnectReason);
void onMqttUnsubscribe(uint16_t);
void onMqttPublish(uint16_t);
void onMqttSubscribe(uint16_t, uint8_t);
void onMqttMessage(char*, char*, AsyncMqttClientMessageProperties, size_t, size_t, size_t);

#endif