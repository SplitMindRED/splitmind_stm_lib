#pragma once

#include "stdint.h"
#include "math.h"
#include "Arduino.h"
#include <Wire.h>

#define INIT_INFO
//#define SHOW_DATA

#define ERROR_INIT      0
#define SUCCSESS        1

#define PWR_MGMT_1		0x6B
#define WHO_AM_I			0x75
#define SMPLRT_DIV		0x19

class MPU9250
{
public:
   MPU9250(uint8_t _device_address);
   uint8_t init();
   void getData(uint8_t* p_byte, uint8_t size);
   void printData();
   void changeSampleRate(int8_t Koeff);

   //range of accelerometer 2g (00), 4g (01), 8g (10), 16g (11) in 3 and 4 bits
   void rangeAccelerometer(int8_t mode);

   //filter strength 1-6 of accelerometer
   void filterAccelerometer(int16_t mode);
   void calibrationAccelerometer(int16_t number_of_tacts);

   //range of gyroscope 
   void rangeGyroscope(int8_t mode);

   //filter strength 1-6 of gyroscope
   void filterGyroscope(int8_t mode);

   //calibration of gyroscope
   void calibrationGyroscope(int16_t number_of_tacts);

   struct Data
   {
      int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, RawAcX, RawAcY, RawAcZ, RawGyX, RawGyY, RawGyZ;
   }data;

private:
   uint8_t device_address;
   uint16_t configuration;
   int32_t AcX_offset = 0, AcY_offset = 0, AcZ_offset = 0;
   int32_t GyX_offset = 0, GyY_offset = 0, GyZ_offset = 0;

};
