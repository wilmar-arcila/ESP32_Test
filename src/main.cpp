#include <Arduino.h>
#include "/home/wilmar/IoT/ESP32_Test/include/global_options.h"

#ifdef TEST_WIFI
#include <mqtt_helper_func.h>
mqttComm mqttC;
uint64_t last_timestamp=0;
#endif

#ifdef TEST_DIMMER
#include <RBDdimmer.h>

#define outputPin  12 
#define zerocross  14 // for boards with CHANGEBLE input pins
#define LAMPMAXVALUE 50
#define INTERVAL 2

int stateL = 1, valLamp;
int DIM4 = 0;
int mainLamp = 0;
int buttonRed = 0; 
int buttonBlue = 0; 
bool setLamp = true;

dimmerLamp dimmer(outputPin, zerocross); //initialase port for dimmer for ESP8266, ESP32, Arduino due boards

void RiseFallLamp(bool RiseFallInt)
{
  if ((RiseFallInt == true) && (mainLamp < LAMPMAXVALUE)) mainLamp+=INTERVAL;
  else if ((RiseFallInt != true) && (mainLamp > 0)) mainLamp-=INTERVAL;
  if (mainLamp >= LAMPMAXVALUE) stateL=0;
  else if (mainLamp <= 0) stateL=1;
}

bool setLampState(int val)
{
  bool ret;
  if (val >= 1) ret = true;
  else ret = false;
  return ret;
}

void readButtonState()
{
  buttonRed = digitalRead(13);
  buttonBlue = digitalRead(15);
  
  if (buttonRed < 1) stateL++;
  if (buttonBlue < 1) stateL--;
  if (stateL < 0) stateL = 0;
  if (stateL > 1) stateL = 1;
}
#endif

#ifdef TEST_TEMP_SENSOR
#include <Wire.h>
#include <AHT25_Sensor.h>

#define AHT25_ADDRESS 0x38
#define SCL 22
#define SDA 21
// Los canales del ADC2 son usados por el módulo WiFi.
// Se debe seleccionar un canal del ADC1
#define ANALOGIN 35           //ADC1_CH7
#define ANALOG_REF_DRY 3155   // Valor del sensor expuesto al aire seco (HR 0%)
#define ANALOG_REF_WET 1645   // Valor del sensor en recipiente con agua (HR 100%)
uint16_t analogValue;
uint8_t soilH_p;
float m;
AHT25 aht;
#endif


/*
https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
*/

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10       /* Time ESP32 will go to sleep (in seconds) */
#define TouchThreshold 40       /* Greater the value, more the sensitivity */

RTC_DATA_ATTR int bootCount = 0;

touch_pad_t touchPin;

#define LOOP_INTERVAL 10000         // Intervalo en ms entre ejecuciones del código Loop()
unsigned long previousMillis = 0;   // Stores last time temperature was published

#define LED_PIN 2
uint8_t ledState;

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

/*
Method to print the touchpad by which ESP32
has been awaken from sleep
*/
void print_wakeup_touchpad(){
  touchPin = esp_sleep_get_touchpad_wakeup_status();

  switch(touchPin)
  {
    case 0  : Serial.println("Touch detected on GPIO 4"); break;
    case 1  : Serial.println("Touch detected on GPIO 0"); break;
    case 2  : Serial.println("Touch detected on GPIO 2"); break;
    case 3  : Serial.println("Touch detected on GPIO 15"); break;
    case 4  : Serial.println("Touch detected on GPIO 13"); break;
    case 5  : Serial.println("Touch detected on GPIO 12"); break;
    case 6  : Serial.println("Touch detected on GPIO 14"); break;
    case 7  : Serial.println("Touch detected on GPIO 27"); break;
    case 8  : Serial.println("Touch detected on GPIO 33"); break;
    case 9  : Serial.println("Touch detected on GPIO 32"); break;
    default : Serial.println("Wakeup not by touchpad"); break;
  }
}

void callback(){
  //placeholder callback function
  Serial.println("Callback function");
}

void setup(){
  Serial.begin(115200);
  delay(1000); //Take some time to open up the Serial Monitor
  
  ++bootCount;  //Increment boot number and print it every reboot
  Serial.println("Boot number: " + String(bootCount));

  pinMode(LED_PIN, OUTPUT);  // LED
  digitalWrite(LED_PIN, LOW);
  ledState=LOW;

  #ifdef TEST_TEMP_SENSOR
  bool sensorInitialized = aht.begin(AHT25_ADDRESS, SDA, SCL);
  Serial.printf("Sensor Initialized: %d\n", sensorInitialized);

  uint8_t stat = aht.readStatus();
  Serial.print(stat, HEX);
  analogValue=0;
  soilH_p=0;
  m = -100.0/(ANALOG_REF_DRY-ANALOG_REF_WET);
  analogSetAttenuation(ADC_11db);
  // analogSetPinAttenuation(LED_PIN, ADC_11db);
  #endif

  #ifdef TEST_DIMMER
  dimmer.begin(NORMAL_MODE, ON); //dimmer initialisation: name.begin(MODE, STATE)
  #endif

  #ifdef TEST_WIFI
  WiFiInit();
  mqttC.mqttInit();
  #endif

  #ifdef TEST_SLEEP
  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  print_wakeup_touchpad();

  touchAttachInterrupt(T3, callback, TouchThreshold);            //Setup interrupt on Touch Pad 3 (GPIO15)  
  esp_sleep_enable_touchpad_wakeup();                            //Wake up source: Touchpad
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); //Wake up source: TIMER -> Set the ESP32 to wake up every TIME_TO_SLEEP seconds
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

  /*
  Next we decide what all peripherals to shut down/keep on
  By default, ESP32 will automatically power down the peripherals
  not needed by the wakeup source, but if you want to be a poweruser
  this is for you. Read in detail at the API docs
  http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
  Left the line commented as an example of how to configure peripherals.
  The line below turns off all RTC peripherals in deep sleep.
  */
  //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  //Serial.println("Configured all RTC Peripherals to be powered down in sleep");

  /*
  Now that we have setup a wake cause and if needed setup the
  peripherals state in deep sleep, we can now start going to
  deep sleep.
  In the case that no wake up sources were provided but deep
  sleep was started, it will sleep forever unless hardware
  reset occurs.
  */
  Serial.println("Going to sleep now");
  delay(1000);
  Serial.flush(); 
  esp_deep_sleep_start();
  #endif
}

void loop(){
  unsigned long currentMillis = millis();
  // Every X number of seconds 
  if (currentMillis - previousMillis >= LOOP_INTERVAL) {
    previousMillis = currentMillis;     // Save the last time the code was executed
    Serial.println("LOOP");
    // if(ledState){
    //   digitalWrite(LED_PIN, LOW);
    //   ledState=LOW;
    //   Serial.println("LED OFF");
    // }
    // else{
    //   digitalWrite(LED_PIN, HIGH);
    //   ledState=HIGH;
    //   Serial.println("LED ON");
    // }
    
    #ifdef TEST_TEMP_SENSOR
    aht.read();
    #ifdef DEBUG
    Serial.printf("Connected: %u\n",aht.isConnected());
    #endif
    Serial.printf("Temperatura: %3.2fºC\n", aht.getTemperature());
    Serial.printf("Humedad: %3.2f%%\n", aht.getHumidity());
    analogValue=analogRead(ANALOGIN);
    soilH_p = m*analogValue+(m*(-ANALOG_REF_WET))+100;
    #ifdef DEBUG
    Serial.printf("analog in: %u\n",analogValue);
    #endif
    Serial.printf("Soil RH: %u%%\n",soilH_p);
    #endif

    #ifdef TEST_DIMMER
    // readButtonState();
    Serial.printf(">> Estado de la lámpara: %d\n", stateL);
    Serial.printf("\t %d%\n", mainLamp);
    dimmer.setPower(mainLamp); // setPower(0-100%);
    RiseFallLamp(setLampState(stateL));
    #endif

    #ifdef TEST_WIFI
    mqttC.setVariable(1, aht.getTemperature(), "Contexto de prueba");
    mqttC.setVariable(2, aht.getHumidity(), "Contexto de prueba 2");
    mqttC.setVariable(3, soilH_p, "Contexto de prueba 3");
    mqttC.publish();
    
    uint64_t tempTimestamp = mqttC.getRxTimestamp();
    if (tempTimestamp > last_timestamp){
      last_timestamp = tempTimestamp;
      singleVariable vTemp=mqttC.getLastMessage();
      Serial.printf("value: %u\n",vTemp.value);
      Serial.printf("context: %s\n",vTemp.varContext);
      Serial.printf("timestamp: %llu\n",last_timestamp);
      if (vTemp.value > 0){
        digitalWrite(LED_PIN, HIGH);
        ledState=HIGH;
        Serial.println("LED ON");
      }
      else{
        digitalWrite(LED_PIN, LOW);
        ledState=LOW;
        Serial.println("LED OFF");
      }
    }
    #endif
  }
}
