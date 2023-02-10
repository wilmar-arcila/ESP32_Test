# ESP32_Test
Prueba de envío de datos de varios sensores usando el protocolo MQTT, un dispositivo ESP32 y la nube de Ubidots
### Tecnologías:  
- Arduino
- ESP32
- MQTT
---
## Archivos de configuración
Los datos sensibles, como contraseñas de red o credenciales de acceso a servidores o nubes, deben ser incluídos en archivos **__XXXX_config.h__**, donde XXXX representa el módulo asociado al archivo de configuración.  
Por ejemplo, para almacenar las contraseñas de la red inalámbrica a la que se va a conectar el dispositivo IoT se crearía el archivo **__wifi_config.h__** con la siguiente estructura:
```C++
const char* const SSID = "SSIDDeLaRedWiFi";
const char* const PASSWD = "ContraseñaDeLaRedWiFi";
const char* const HOSTNAME = "NombreAsignadoAlDispositivoIoT";
```
y para almacenar la información MQTT (credenciales de Ubidots, variables a publicar, tópico, host, puerto, etc.) se crearía el archivo **__mqtt_config.h__** con una estructura similar a la siguiente:
```C++
#define MQTT_HOST "HostDelBrokerMqtt"
#define MQTT_PORT 1883
#define MQTT_USERNAME "UsernameAsignadoPorElHostMqtt"
#define MQTT_PASSWORD "PasswordDelHostMqtt"
#define MQTT_SENSOR_TOPIC "/v2.0/devices/"
#define MQTT_COMMAND_TOPIC "/v1.6/devices/"
#define MQTT_SENSOR_DEVICE "esp32"
#define MQTT_COMMAND_DEVICE "control/"
#define MQTT_COMMAND_VAR "esp32"
#define MQTT_TOTAL_VARS 3
#define MQTT_SENSOR1_VAR "temp"
#define MQTT_SENSOR2_VAR "airH"
#define MQTT_SENSOR3_VAR "soilH"
```
Estos archivos deben almacenarse en la carpeta correspondiente a su respectivo módulo e incluirse en el archivo de cabeceras del mismo.

---
### Notas
- En el archivo __include/global_options.h__ se puede seleccionar el tipo de test que se desea realizar (o una combinación de ellos):
    - DEBUG
    - TEST WIFI
    - TEST SLEEP MODES
    - TEST SENSOR TEMPERATURA