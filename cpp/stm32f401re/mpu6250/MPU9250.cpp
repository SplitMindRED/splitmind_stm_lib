#include "MPU9250.h"

MPU9250::MPU9250(uint8_t _device_address)
{
   device_address = _device_address;
}

uint8_t MPU9250::init()
{
   Wire.beginTransmission(device_address);
   Wire.write(0x6B);  // PWR_MGMT_1 register
   Wire.write(0);     // set to zero (wakes up the MPU-6050)
   Wire.endTransmission(true);

   delay(10);

   Wire.beginTransmission(device_address);
   Wire.write(WHO_AM_I);
   Wire.endTransmission();
   Wire.requestFrom((uint8_t)device_address, (uint8_t)1);
   int8_t a = Wire.read();

#ifdef INIT_INFO
   //0x71
   Serial.print("MPU9250 WHO AM I: ");
   Serial.println(a, HEX);
#endif

   if (a == 0x71)
   {
#ifdef INIT_INFO
      Serial.println("MPU9250 INIT SUCCSESS!!!");
#endif      
      // return SUCCSESS;
   }
   else
   {
#ifdef INIT_INFO
      Serial.println("MPU9250 INIT FAILURE!!!");
#endif
      return ERROR_INIT;
   }

   changeSampleRate(9); //update rate of sensor register. 1000/(1 + 9)= 100 Hz
   filterAccelerometer(2);
   rangeAccelerometer(0b00011000);
   calibrationAccelerometer(200);

   filterGyroscope(2);
   rangeGyroscope(0b00011000);
   calibrationGyroscope(200);

   return SUCCSESS;
}

void MPU9250::getData(uint8_t* p_byte, uint8_t size)
{
   Wire.beginTransmission(device_address);
   Wire.write(0x3B);                           // starting with register 0x3B (ACCEL_XOUT_H)
   Wire.endTransmission(false);
   Wire.requestFrom((uint8_t)device_address, (uint8_t)14, (uint8_t)1); // request a total of 14 registers

   data.RawAcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
   data.RawAcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
   data.RawAcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

   data.Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

   data.RawGyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
   data.RawGyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
   data.RawGyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

   data.AcX = data.RawAcX - AcX_offset;
   data.AcY = data.RawAcY - AcY_offset;
   data.AcZ = data.RawAcZ - AcZ_offset;
   data.GyX = data.RawGyX - GyX_offset;
   data.GyY = data.RawGyY - GyY_offset;
   data.GyZ = data.RawGyZ - GyZ_offset;

   uint8_t* p_data = (uint8_t*)&data;

   for (uint8_t var_counter = 0; var_counter < 6; var_counter++)
   {
      //skip temperature data
      if (var_counter == 3)
      {
         // data.Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
         p_data++;
         p_data++;
      }

      // //MSB
      // p_byte++;
      // *p_byte = Wire.read();
      // //LSB
      // p_byte--;
      // *p_byte = Wire.read();
      // p_byte = p_byte + 2;

      *p_byte++ = *p_data++;
      *p_byte++ = *p_data++;
   }

   printData();
}

void MPU9250::printData()
{
#ifdef SHOW_DATA
   Serial.print("AcX = "); Serial.print(data.AcX);
   Serial.print(" | AcY = "); Serial.print(data.AcY);
   Serial.print(" | AcZ = "); Serial.print(data.AcZ);
   Serial.print(" | GyX = "); Serial.print(data.GyX);
   Serial.print(" | GyY = "); Serial.print(data.GyY);
   Serial.print(" | GyZ = "); Serial.println(data.GyZ);
#endif // SHOW_DATA
}

void MPU9250::changeSampleRate(int8_t Koeff)
{
   Wire.beginTransmission(device_address);
   Wire.write(SMPLRT_DIV);
   Wire.write(Koeff);
   Wire.endTransmission();
   delay(1);
}

void MPU9250::rangeAccelerometer(int8_t mode)
{
   Wire.beginTransmission(device_address);
   Wire.write(28);
   Wire.write(mode);
   Wire.endTransmission();
   delay(1);
}

void MPU9250::filterAccelerometer(int16_t mode)
{
   Wire.beginTransmission(device_address);
   Wire.write(29);
   Wire.write(mode);
   Wire.endTransmission();
   delay(1);
}

void MPU9250::calibrationAccelerometer(int16_t number_of_tacts)
{
   Serial.println("Calibration of accelerometer... ");

   for (int16_t i = 0; i < number_of_tacts; i++)
   {
      Wire.beginTransmission(device_address);
      Wire.write(0x3B);								         // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission();
      Wire.requestFrom((uint8_t)device_address, (uint8_t)6);			   // request a total of 6 bytes
      AcX_offset += Wire.read() << 8 | Wire.read();	// 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
      AcY_offset += Wire.read() << 8 | Wire.read();	// 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ_offset += Wire.read() << 8 | Wire.read();	// 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      delay(10);
   }

   AcX_offset = AcX_offset / number_of_tacts;
   AcY_offset = AcY_offset / number_of_tacts;
   AcZ_offset = AcZ_offset / number_of_tacts;

   Serial.println("Calibration of accelerometer complete: ");
   Serial.print("AcX_offset = "); Serial.print(AcX_offset);
   Serial.print(" | AcY_offset = "); Serial.print(AcY_offset);
   Serial.print(" | AcZ_offset = "); Serial.println(AcZ_offset);
}

void MPU9250::rangeGyroscope(int8_t mode)
{
   Wire.beginTransmission(device_address);
   Wire.write(27);
   Wire.write(mode);
   Wire.endTransmission();
   delay(1);
}

void MPU9250::filterGyroscope(int8_t mode)
{
   Wire.beginTransmission(device_address);
   Wire.write(26);
   Wire.write(mode);
   Wire.endTransmission();
   delay(1);
}

void MPU9250::calibrationGyroscope(int16_t number_of_tacts)
{
   Serial.println("Calibration of gyroscope... ");
   for (int16_t i = 0; i < number_of_tacts; i++)
   {
      Wire.beginTransmission(device_address);
      Wire.write(0x43);									         // starting with register 0x43 (GYRO_XOUT_L)
      Wire.endTransmission();
      Wire.requestFrom((uint8_t)device_address, (uint8_t)6);				   // request a total of 6 bytes
      GyX_offset += Wire.read() << 8 | Wire.read();		// 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)    
      GyY_offset += Wire.read() << 8 | Wire.read();		// 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      GyZ_offset += Wire.read() << 8 | Wire.read();		// 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
      delay(10);
   }

   GyX_offset = GyX_offset / number_of_tacts;
   GyY_offset = GyY_offset / number_of_tacts;
   GyZ_offset = GyZ_offset / number_of_tacts;

   Serial.println("Calibration of gyroscope complete: ");
   Serial.print("GyX_offset = "); Serial.print(GyX_offset);
   Serial.print(" | GyY_offset = "); Serial.print(GyY_offset);
   Serial.print(" | GyZ_offset = "); Serial.println(GyZ_offset);
}
