#pragma once

#include "Eigen/Core"
#include "gpio.h"
#include "i2c.h"
#include "main.h"
#include "stdint.h"
#include "stm32f401re/splitmind_f401_hal_lib.h"
#include "stm32f401re/usart_debug/usart_debug.h"
#include "tim.h"
#include "usart.h"

#define INIT_INFO
// #define INIT_CALIBRATION
#define SHOW_DATA

#define ERROR_INIT 0
#define SUCCSESS 1

#define MPU6250_ADDRESS 0xD0

#define PWR_MGMT_1 0x6B
#define WHO_AM_I 0x75
#define SMPLRT_DIV 0x19

extern bool is_new_data;
extern uint8_t data_frame[14];
extern uint8_t data_send[14];
extern uint8_t data_frame_raw[14];
extern uint8_t recv_size;
extern uint8_t send_size;

extern void changeConfig(void);
extern void getData(void);
extern void printData(void);

//interruption after tx
extern void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c);

//recieve data interrupt
extern void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c);

struct MPUData
{
  float AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, RawAcX, RawAcY, RawAcZ, RawGyX, RawGyY, RawGyZ;
  float roll;
  float pitch;
  float acc_roll;
  float acc_pitch;
  float yaw;
  float droll;
  float dpitch;
  unsigned long t_prev;
  Eigen::Vector2f dOri;
};

class MPU6250
{
public:
  MPU6250(uint8_t _device_address);

  uint8_t init();
  // void getData(uint8_t* p_byte, uint8_t size);
  void getData();
  void printData();
  void printRawData();
  void changeSampleRate(int8_t Koeff);
  void plotData();

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

  void complimentaryFilter();
  void lowPassFilter();

  MPUData data;

private:
  uint8_t _device_address;
  uint16_t _configuration;
  float _AcX_offset = 0, _AcY_offset = 0, _AcZ_offset = 0;
  float _GyX_offset = 0, _GyY_offset = 0, _GyZ_offset = 0;

  Eigen::Matrix<float, 2, 3> R_gyro;
};