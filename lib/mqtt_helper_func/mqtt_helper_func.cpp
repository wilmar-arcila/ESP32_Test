//
//    FILE: mqtt_helper_func.cpp
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

#include "/home/wilmar/IoT/ESP32_Test/include/global_options.h"
#include <mqtt_helper_func.h>

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
mqttComm* mqttCommObject;


mqttComm::mqttComm(){
    _mqttConnected=false;
    _sensorId=WiFi.macAddress();
    _status="operative";
    _rxTimestamp=0;
    _txTimestamp=0;
    _ntpServer = "pool.ntp.org";
    _receivedVariable = {0,"Just created"};
    _MQTT_SENSOR1_VAR = {0,"Just created"};
    _MQTT_SENSOR2_VAR = {0,"Just Created"};
    _MQTT_SENSOR3_VAR = {0,"Just Created"};
    _varPointers[0]=&_receivedVariable;
    _varPointers[1]=&_MQTT_SENSOR1_VAR;
    _varPointers[2]=&_MQTT_SENSOR2_VAR;
    _varPointers[3]=&_MQTT_SENSOR3_VAR;
    mqttCommObject=this;
}

void mqttComm::mqttInit(){
    // mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(mqttComm::mqttConnectToBroker));
    // wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(WiFiConnect));

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onSubscribe(onMqttSubscribe);
    mqttClient.onUnsubscribe(onMqttUnsubscribe);
    mqttClient.onPublish(onMqttPublish);
    mqttClient.onMessage(onMqttMessage);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);

     #ifdef DEBUG                  
    Serial.printf("Creando objetos: StaticJsonDocument<%d>\n", CAPACITY);
    #endif
    _filter = StaticJsonDocument<CAPACITY>();

    // The filter: it contains "true" for each value we want to keep
    _filter["value"] = true;
    _filter["timestamp"] = true;
    _filter["context"] = true;

    mqttConnectToBroker();
    configTime(0, 0, _ntpServer);
    _epochTimems = getTime();
    #ifdef DEBUG
    Serial.printf("Epoch Time: %llu\n",_epochTimems);
    #endif
    _epochTimems*=1000;
    #ifdef DEBUG
    Serial.printf("Epoch Time ms: %llu\n",_epochTimems);
    #endif
}

// void mqttComm::printMessage(biofarmMessage *msg){
//     Serial.println("DESERIALIZED MSG:");
//     Serial.printf("\tvalue: %3.2f\n", (*msg).value);
//     Serial.printf("\ttimestamp: %u\n", (*msg).timestamp/1000);
//     Serial.printf("\tcontext: %s\n", (*msg).context.c_str());
// }

bool mqttComm::setVariable(uint8_t id, float value, String context){
    if ((id > MQTT_TOTAL_VARS) || (id < 1)){return false;}
    _varPointers[id]->value = value;
    _varPointers[id]->varContext = context;
    return true;
}

singleVariable mqttComm::getLastMessage(){
    return *_varPointers[0];
}

void mqttComm::publish(){
    StaticJsonDocument<CAPACITY>_doc = mqttCreateMessage();
    char payload[CAPACITY];
    serializeJson(_doc, payload);
    // Publish an MQTT message on topic
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_SENSOR_TOPIC MQTT_SENSOR_DEVICE, 1, true, (const char *)&payload);

    #ifdef DEBUG
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_SENSOR_TOPIC MQTT_SENSOR_DEVICE, packetIdPub1);
    Serial.printf(" Message:\n%s\n", payload);
    char msg_d[(CAPACITY)*2];
    serializeJsonPretty(_doc, msg_d);               
    Serial.printf(" Message:\n%s\n", msg_d);
    #endif
}

void mqttComm::mqttProcessMsg(char* msg){
    String _msg = String(msg); 
    String _json = _msg.substring(0,_msg.lastIndexOf("}")+1);
    #ifdef DEBUG
    Serial.printf("MSG: %s\n", _json.c_str());
    #endif

    // char* _json; (the input. Required)
    // size_t inputLength; (Optional)
    // DeserializationOption::Filter filter. (Optional)
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, _json.c_str(), _json.length(), DeserializationOption::Filter(_filter));
    // DeserializationError error = deserializeJson(doc, _json.c_str(), _json.length());
    if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
    }
    _varPointers[0]->value = doc["value"];
    _varPointers[0]->varContext = doc["context"].as<String>();
    _rxTimestamp = doc["timestamp"];
    return;
}

//********************************************************************//
//*************************PRIVATE METHODS****************************//

void mqttComm::mqttConnectToBroker() {
    #ifdef DEBUG
    Serial.println("Connecting to MQTT BROKER...");
    #endif
    mqttClient.connect();
}

StaticJsonDocument<CAPACITY> mqttComm::mqttCreateMessage(){
    StaticJsonDocument<CAPACITY> _doc;
    JsonObject v1 = _doc.createNestedObject(MQTT_SENSOR1_VAR);
    v1["value"]   = _varPointers[1]->value;
    JsonObject c1 = v1.createNestedObject("context");
    c1["extra"] = _varPointers[1]->varContext;
    c1["sensor_id"] = _sensorId;

    JsonObject v2 = _doc.createNestedObject(MQTT_SENSOR2_VAR);
    v2["value"]   = _varPointers[2]->value;
    JsonObject c2 = v2.createNestedObject("context");
    c2["extra"] = _varPointers[2]->varContext;
    c2["sensor_id"] = _sensorId;

    JsonObject v3 = _doc.createNestedObject(MQTT_SENSOR3_VAR);
    v3["value"]   = _varPointers[3]->value;
    JsonObject c3 = v3.createNestedObject("context");
    c3["extra"] = _varPointers[3]->varContext;
    c3["sensor_id"] = _sensorId;
    
    _txTimestamp =_epochTimems+millis();
    #ifdef DEBUG
    uint64_t temp_t =getTime();
    temp_t*=1000;    
    Serial.printf("Epoch Time ms   : %llu\n",temp_t);
    Serial.printf("Epoch Time local: %llu\n",_txTimestamp);
    int16_t diff_t = temp_t - _txTimestamp;
    Serial.printf("Diferencia      :         %d\n",diff_t);
    #endif
    _doc["timestamp"]=_txTimestamp;
    return _doc;
}

// Function that gets current epoch time
unsigned long mqttComm::getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    #ifdef DEBUG
    Serial.println("Failed to obtain time");
    #endif
    return(0);
  }
  time(&now);
  return now;
}

//********************************************************************//
//*****************************ISR METHODS****************************//
// Estos métodos no pueden ser miembros de la clase.
// Ver: https://isocpp.org/wiki/faq/pointers-to-members
//********************************************************************//

void onMqttConnect(bool sessionPresent) {
    Serial.println("Connected to MQTT BROKER.");
    // _mqttConnected=true;
    Serial.printf("Session present: %u\n", sessionPresent);
    uint16_t packetIdSub = mqttClient.subscribe(MQTT_COMMAND_TOPIC MQTT_COMMAND_DEVICE MQTT_COMMAND_VAR, 0);
    Serial.printf("Subscribing at QoS 0, packetId: %u\n", packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Serial.println("Disconnected from MQTT.");
    if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
    }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
    #ifdef DEBUG
    Serial.println("Subscribe acknowledged.");
    Serial.printf("  packetId: %d\n", packetId);
    Serial.printf("  qos: %d\n", qos);
    #endif
}

void onMqttUnsubscribe(uint16_t packetId) {
    #ifdef DEBUG
    Serial.println("Unsubscribe acknowledged.");
    Serial.print("  packetId: ");
    Serial.println(packetId);
    #endif
}

void onMqttPublish(uint16_t packetId) {
    #ifdef DEBUG
    Serial.print("Publish acknowledged.");
    Serial.printf("  packetId: %d\n", packetId);
    #endif
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
    #ifdef DEBUG
    Serial.println("Publish received.");
    Serial.printf("  topic: %s\n", topic);
    Serial.printf("  qos: %u\n", properties.qos);
    Serial.printf("  dup: %u\n", properties.dup ? "true" : "false");
    Serial.printf("  retain: %u\n", properties.retain ? "true" : "false");
    Serial.printf("  len: %u\n", len);
    Serial.printf("  index: %u\n", index);
    Serial.printf("  total: %u\n", total);
    // Serial.printf("  payload: %s\n", payload);
    #endif
    mqttCommObject->mqttProcessMsg(payload);
    // Poner aquí alguna función que se desee ejecutar cuando llegue un mensaje.
    // xxxxxxxxxxxx
}