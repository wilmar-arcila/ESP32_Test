//
//    FILE: AHT25.h
//  AUTHOR: Wilmar Arcila
// VERSION: 0.0.1
//    DATE: 2022-06-09
// PURPOSE: Arduino library for the AHT25 temperature and humidity sensor
//
//  *** BASED ON LIBRARY SHT31 BY ROB TILLAART https://github.com/RobTillaart/SHT31 ***
//
//  HISTORY:
//  0.0.1   2022-06-09  initial version
#pragma once

#include "Arduino.h"
#include "Wire.h"


#define AHT31_LIB_VERSION             (F("0.0.1"))

#ifndef AHT_DEFAULT_ADDRESS   
#define AHT_DEFAULT_ADDRESS           0x38
#endif

// fields readStatus
#define AHT25_STATUS_BUSY        (0x80)   // 10000000
#define AHT25_STATUS_RETAIN      (0x77)   // 01110111
#define AHT25_STATUS_CALIBRATED  (0x08)   // 00001000

// error codes
#define AHT25_OK                  0x00
#define AHT25_ERR_WRITECMD        0x81
#define AHT25_ERR_READBYTES       0x82
#define AHT25_ERR_NOT_CONNECT     0x84
#define AHT25_ERR_CRC             0x85


class AHT25
{
public:
  AHT25();

#if defined(ESP8266) || defined(ESP32)
  bool begin(const uint8_t address, uint8_t dataPin, uint8_t clockPin);
  // use AHT_DEFAULT_ADDRESS
  bool begin(const uint8_t dataPin, const uint8_t clockPin);
#endif
  bool begin(const uint8_t address,  TwoWire *wire = &Wire);
  // use AHT_DEFAULT_ADDRESS
  bool begin(TwoWire *wire = &Wire);

  // blocks 15 milliseconds + actual read + math
  bool read();

  // check sensor is initialized
  bool isInitialized();

  // check sensor is reachable over I2C
  bool isConnected();

  // details see datasheet; summary in AHT25_Sensor.cpp file
  uint8_t readStatus();

  // lastRead is in milliSeconds since start
  uint32_t lastRead() { return _lastRead; };

  bool reset();

  float    getHumidity()       { return ((float)_rawHumidity    / 1048576) * 100.0; };
  float    getTemperature()    { return ((float)_rawTemperature / 1048576) * 200.0 - 53.08; };
  uint16_t getRawHumidity()    { return _rawHumidity; };
  uint16_t getRawTemperature() { return _rawTemperature; };


  // ASYNC INTERFACE
  bool readSensorValues();

  int getError(); // clears error flag

private:
  uint8_t crc8(const uint8_t *data, uint8_t len);
  bool writeCmd(uint8_t cmd);
  bool writeCmd(uint8_t addr, uint8_t cmd);
  bool writeCmd(uint8_t addr, uint8_t cmd, uint16_t data);
  bool readBytes(uint8_t n, uint8_t *val);
  bool initSensor(void);

  TwoWire* _wire;

  uint8_t   _address;
  uint32_t  _lastRead;
  uint32_t  _lastRequest;   // for async interface

  uint32_t _rawHumidity;
  uint32_t _rawTemperature;

  uint8_t _error;
};

// -- END OF FILE --
