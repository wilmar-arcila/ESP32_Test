//
//    FILE: AHT25.cpp
//  AUTHOR: Wilmar Arcila
// VERSION: 0.0.1
//    DATE: 2022-06-09
// PURPOSE: Arduino library for the AHT25 temperature and humidity sensor
//
//  *** BASED ON LIBRARY SHT31 BY ROB TILLAART https://github.com/RobTillaart/SHT31 ***
//
//  HISTORY:
//  0.0.1   2022-06-09  initial version


#include "AHT25_Sensor.h"
#include "/home/wilmar/IoT/ESP32_Test/include/global_options.h"


// SUPPORTED COMMANDS - single shot mode only
#define AHT25_TRIGGER_MEASURE   0xAC
#define AHT25_SOFT_RESET        0XBA
#define AHT25_READ_STATUS       0x71


AHT25::AHT25()
{
  _address        = 0;
  _lastRead       = 0;
  _rawTemperature = 0;
  _rawHumidity    = 0;
  _error          = AHT25_OK;
}

/**************************************************************************/
/*
    begin()
    Initialize I2C & sensor
    NOTE:
    - call this function before doing anything else!!!
*/
/**************************************************************************/
#if defined(ESP8266) || defined(ESP32)
bool AHT25::begin(const uint8_t address, const uint8_t dataPin, const uint8_t clockPin)
{
  if (address != 0x38)
  {
    return false;
  }
  _address = address;
  Wire.begin();
  _wire = &Wire;
  if ((dataPin < 255) && (clockPin < 255))
  {
    _wire->begin(dataPin, clockPin);
  } else {
    _wire->begin();
  }
  Wire.setClock(100000);

  if(reset() == false){return false;}

  return initSensor();
}

bool AHT25::begin(const uint8_t dataPin, const uint8_t clockPin)
{
  return begin(AHT_DEFAULT_ADDRESS, dataPin, clockPin);
}
#endif

bool AHT25::begin(const uint8_t address,  TwoWire *wire)
{
  if (address != 0x38)
  {
    return false;
  }
  _address = address;
  _wire    = wire;
  _wire->begin();
   if(reset() == false){return false;}

    return initSensor();
}

bool AHT25::begin(TwoWire *wire)
{
  return begin(AHT_DEFAULT_ADDRESS, wire);
}


bool AHT25::read()
{
  if (writeCmd(_address, AHT25_TRIGGER_MEASURE, 0x3300) == false)
  {
    return false;
  }
  delay(80);
  while( (readStatus() & AHT25_STATUS_BUSY) != 0){delay(10);}
  return readSensorValues();
}


bool AHT25::isConnected()
{
  _wire->beginTransmission(_address);
  int rv = _wire->endTransmission();
  if (rv != 0) _error = AHT25_ERR_NOT_CONNECT;
  return (rv == 0);
}

#ifdef doc
// bit - description
// ==================
//   7 Busy indication
//    '1': Equipment is busy, in measurement mode
//    '0': Equipment is idle, in hibernation state
// 6:5 Retain
//   4 Retain
//   3 CAL Enable
//    '1': Calibrated
//    '0': Uncalibrated
// 2:0 Retain
#endif


uint8_t AHT25::readStatus()
{
  uint8_t status = 0;
  
  if (writeCmd(AHT25_READ_STATUS) == false)
  {
    return 0xFF;
  }
  // 8 bit status
  if (readBytes(1, (uint8_t*) &status) == false)
  {
    return 0xFF;
  }

  return (uint8_t)status;
}


bool AHT25::reset()
{
  bool b = writeCmd(AHT25_SOFT_RESET);
  if (b == false)
  {
    return false;
  }
  delay(1);
  return true;
}

bool AHT25::readSensorValues()
{
    uint8_t buffer[7];
    if (readBytes(7, (uint8_t*) &buffer[0]) == false)
    {
        return false;
    }
    #ifdef DEBUG
    Serial.printf("Buffer: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6]);
    Serial.printf("CRC: 0x%X\t Buffer[6]: 0x%X\n", crc8(buffer, 6), buffer[6]);
    #endif

    if (buffer[6] != crc8(buffer, 6)) 
    {
      _error = AHT25_ERR_CRC;
      return false;
    }
    
    _rawHumidity    = ((buffer[1] << 12) + (buffer[2] << 4) + (buffer[3] >> 4));
    _rawTemperature = (((buffer[3] & 0x0F) << 16) + (buffer[4] << 8) + buffer[5]);

    #ifdef DEBUG
    Serial.printf("raw Hum: 0x%X %f\n",_rawHumidity,(float)_rawHumidity);
    Serial.printf("raw Temp: 0x%X %f\n",_rawTemperature,(float)_rawTemperature);
    #endif

    _lastRead = millis();

    return true;
}


int AHT25::getError()
{
  int rv = _error;
  _error = AHT25_OK;
  return rv;
}


//////////////////////////////////////////////////////////
bool AHT25::initSensor()
{
    if((readStatus() && 0x18) != 0x18){
        if (writeCmd(_address,0x1B, 0x0000) == false){return false;}
        delay(1);
        if (writeCmd(_address,0x1C, 0x0000) == false){return false;}
        delay(1);
        if (writeCmd(_address,0x1E, 0x0000) == false){return false;}
        delay(1);
    }
    delay(10);
    return true;
}


uint8_t AHT25::crc8(const uint8_t *data, uint8_t len) 
{
  // CRC-8 formula from page 14 of SHT spec pdf
  const uint8_t POLY(0x31);
  uint8_t crc(0xFF);

  for (uint8_t j = len; j; --j) 
  {
    crc ^= *data++;

    for (uint8_t i = 8; i; --i) 
    {
      crc = (crc & 0x80) ? (crc << 1) ^ POLY : (crc << 1);
    }
  }
  return crc;
}

/**************************************************************************/
/* returned value by "Wire.endTransmission()":
      - 0 success
      - 1 data too long to fit in transmit data buffer
      - 2 received NACK on transmit of address
      - 3 received NACK on transmit of data
      - 4 other error
*/
/**************************************************************************/
bool AHT25::writeCmd(uint8_t cmd)
{
  return writeCmd(_address, cmd);
}

bool AHT25::writeCmd(uint8_t addr, uint8_t cmd)
{
  _wire->beginTransmission(addr);
  #ifdef DEBUG
  Serial.printf("Wire:begin transmission: 0x%X\n",addr);
  #endif
  _wire->write(cmd);
  #ifdef DEBUG
  Serial.printf("Wire:send command: 0x%X\n",cmd);
  #endif
  uint8_t temperror=_wire->endTransmission();
  if (temperror != 0)
  {
    _error = AHT25_ERR_WRITECMD;
    #ifdef DEBUG
    Serial.printf("Wire:send command:ERROR:%u\n", temperror);
    #endif
    return false;
  }
  #ifdef DEBUG
  Serial.println("Wire:send command:OK");
  #endif
  return true;
}

bool AHT25::writeCmd(uint8_t addr, uint8_t cmd, uint16_t data)
{
  _wire->beginTransmission(addr);
  #ifdef DEBUG
  Serial.printf("Wire:begin transmission: 0x%X\n",addr);
  #endif
  _wire->write(cmd);
  _wire->write(data >> 8);
  _wire->write(data && 0xFF);
  #ifdef DEBUG
  Serial.printf("Wire:send command: 0x%X - 0x%X\n", cmd, data);
  #endif
  uint8_t temperror=_wire->endTransmission();
  if (temperror != 0)
  {
    _error = AHT25_ERR_WRITECMD;
    #ifdef DEBUG
    Serial.printf("Wire:send command:ERROR:%u\n", temperror);
    #endif
    return false;
  }
  #ifdef DEBUG
  Serial.println("Wire:send command:OK");
  #endif
  return true;
}


bool AHT25::readBytes(uint8_t n, uint8_t *val)
{
  int rv = _wire->requestFrom(_address, (uint8_t) n);
  if (rv == n)
  {
    for (uint8_t i = 0; i < n; i++)
    {
      val[i] = _wire->read();
    }
    return true;
  }
  _error = AHT25_ERR_READBYTES;
  return false;
}

// -- END OF FILE --
