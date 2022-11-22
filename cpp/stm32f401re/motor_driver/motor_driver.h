#pragma once

#include "Eigen/Core"
#include "gpio.h"
#include "main.h"
#include "stdint.h"
#include "stm32f401re/splitmind_f401_hal_lib.h"
#include "stm32f401re/usart_debug/usart_debug.h"
#include "tim.h"
#include "usart.h"

// PC5 and PC6 exti
extern long ticks[2];

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

struct MotorCmd
{
  float u;
  float M_ext;
};

struct MotorData
{
  float q;
  float q_deg;
  float w;
  float dw;
  float I;
  float ew;
  float edw;
  float eI;
  long ticks;
  float M;
  float eM_ext;
};

struct MotorParameters
{
  float L;
  float R;
  float Kw;
  float Km;
  float J;
  float Lam;
  uint32_t ticks_per_round;
};

struct KalmanFilter
{
  Eigen::Vector3f x;
  Eigen::Vector<float, 5> u;
  Eigen::Matrix3f A_hat;
  Eigen::Matrix<float, 3, 5> B_hat;
  Eigen::Matrix3f Kf;
  float Mext_plot;
};

struct PinStruct
{
  GPIO_TypeDef* port;
  uint16_t pin;
  TIM_HandleTypeDef* tim;
  uint32_t tim_channel;
};

class MotorDriver
{
public:
  MotorDriver(char* m_name);
  ~MotorDriver();

  void standBy(bool flag);
  void shortBreak(void);
  void setVoltage(float u);
  void setMext(float M_ext);
  void setParams(MotorParameters params);
  float generateCos(unsigned long t_start, float A, float B, float w, float fi);
  void evalSensorData();
  MotorData getData();
  void setupKF();
  void runKalmanFilter();
  Eigen::Vector3f getKalmanState();
  void torqueControl(float tau_des);

  unsigned long time_start = 0;
  unsigned long t_prev = 0;

  PinStruct in1;
  PinStruct in2;
  PinStruct pwm;
  PinStruct stndby;
  int8_t side_sign = 1;
  int8_t motor_num = 0;
  char* name;

private:
  MotorParameters _param = {0};
  MotorCmd _cmd = {0};
  MotorData _data = {0};
  KalmanFilter _kf = {};
  float _dt = 0;

  bool is_start = 1;
  unsigned long t_start = 0;
  float q_prev = 0;
  float w_prev = 0;
};
