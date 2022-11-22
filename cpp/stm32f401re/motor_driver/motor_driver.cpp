#include "motor_driver.h"

long ticks[2] = {0, 0};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_6) // A
  {
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 1) // A1
    {
      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == 0) // B0
      {
        ticks[0]++;
      }
      else // B1
      {
        ticks[0]--;
      }
    }
    else // A0
    {
      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == 0) // B0
      {
        ticks[0]--;
      }
      else // B1
      {
        ticks[0]++;
      }
    }
  }
  else if (GPIO_Pin == GPIO_PIN_5) // B
  {
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == 1) // B1
    {
      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 0) // A0
      {
        ticks[0]--;
      }
      else // A1
      {
        ticks[0]++;
      }
    }
    else // B0
    {
      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 0) // A0
      {
        ticks[0]++;
      }
      else // A1
      {
        ticks[0]--;
      }
    }
  }

  //-------------------------------------------------------

  if (GPIO_Pin == GPIO_PIN_12) // A
  {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 1) // A1
    {
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 0) // B0
      {
        ticks[1]++;
      }
      else // B1
      {
        ticks[1]--;
      }
    }
    else // A0
    {
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 0) // B0
      {
        ticks[1]--;
      }
      else // B1
      {
        ticks[1]++;
      }
    }
  }
  else if (GPIO_Pin == GPIO_PIN_11) // B
  {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 1) // B1
    {
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 0) // A0
      {
        ticks[1]--;
      }
      else // A1
      {
        ticks[1]++;
      }
    }
    else // B0
    {
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 0) // A0
      {
        ticks[1]++;
      }
      else // A1
      {
        ticks[1]--;
      }
    }
  }
}

MotorDriver::MotorDriver(char* m_name)
{
  time_start = 0;
  name = m_name;

  _param.L = 1.0551;
  _param.R = 5.9043;
  _param.Kw = 1.9437;
  _param.Km = 1.9437;
  _param.J = 0.0085;
  _param.Lam = 0.0020;
  _param.ticks_per_round = 1;

  _kf.A_hat = Eigen::Matrix3f::Zero();
  _kf.B_hat = Eigen::Matrix<float, 3, 5>::Zero();
  _kf.Kf = Eigen::Matrix3f::Zero();
  _kf.x = Eigen::Vector3f::Zero();

  // _data = (const struct MotorData){ 0 };
  _data.w = 0.0;
  _data.dw = 0.0;
  _data.M = 0.0;

  _cmd.M_ext = 0.0;
  _cmd.u = 0.0;
}

MotorDriver::~MotorDriver()
{
}

void MotorDriver::setParams(MotorParameters params)
{
  _param = params;
}

void MotorDriver::setVoltage(float percent)
{
  _cmd.u = percent / 100.0 * 6.0;

  percent = percent * side_sign;

  if (percent > 0.0)
  {
    _cmd.M_ext = -12.3e-3;
    HAL_GPIO_WritePin(in1.port, in1.pin, GPIO_PIN_RESET); // PA7 AIN1
    HAL_GPIO_WritePin(in2.port, in2.pin, GPIO_PIN_SET);   // PC7 AIN2
  }
  else
  {
    _cmd.M_ext = 12.3e-3;
    HAL_GPIO_WritePin(in1.port, in1.pin, GPIO_PIN_SET);   // PA7 AIN1
    HAL_GPIO_WritePin(in2.port, in2.pin, GPIO_PIN_RESET); // PC7 AIN2
  }

  if (abs(percent) < 0.01)
  {
    _cmd.M_ext = 0.0;
  }

  uint32_t pulse = (uint32_t)((float)256 * abs(percent) / (float)100.0);
  // uint32_t pulse = abs(percent);

  // UART_printStr("Pulse:");
  // UART_printLn(pulse);

  __HAL_TIM_SetCompare(pwm.tim, pwm.tim_channel, pulse);
}

void MotorDriver::setMext(float M_ext)
{
  _cmd.M_ext = M_ext;
}

void MotorDriver::standBy(bool flag)
{
  if (flag == 0)
  {
    HAL_GPIO_WritePin(stndby.port, stndby.pin, GPIO_PIN_SET); // PA6 - STNDBY (move)
  }
  else
  {
    HAL_GPIO_WritePin(stndby.port, stndby.pin, GPIO_PIN_RESET); // PA6 - STNDBY (no move)
  }
}

void MotorDriver::shortBreak()
{
  UART_printStrLn("Short break");
  HAL_GPIO_WritePin(in1.port, in1.pin, GPIO_PIN_SET); // PA7 AIN1
  HAL_GPIO_WritePin(in2.port, in2.pin, GPIO_PIN_SET); // PC7 AIN2
}

float MotorDriver::generateCos(unsigned long t_start, float A, float B, float w, float fi)
{
  float u = 0;
  unsigned long t = getStMcs();

  u = A * cos(w * (float)(t - t_start) * 2.0 * 3.1415 / 1000000.0 + fi) + B;

  return u;
}

static unsigned long getTime()
{
  return getStMcs();
}

void MotorDriver::evalSensorData()
{
  static float f_w = 1.0;
  static float f_dw = 1.0;

  if (is_start)
  {
    is_start = 0;
    t_start = getStMcs(); //mcs
    t_prev = t_start;
    _data.w = 0.0;
    _data.dw = 0.0;

    return;
  }

  _data.ticks = side_sign * ticks[motor_num];

  unsigned long t = getStMcs();

  float dt = (float)(t - t_prev) / 1e6;                                        //sec
  _data.q = (float)_data.ticks / (float)_param.ticks_per_round * 2.0 * 3.1415; //q in rad
  // _data.q = (float)ticks[0] / (float)_param.ticks_per_round * 360.0; //q in deg

  // _data.w = (float)(_data.q - q_prev) / dt;
  _data.w = _data.w * (1.0 - f_w) + f_w * (float)(_data.q - q_prev) / dt;
  // _data.dw = (_data.w - w_prev) / dt;
  _data.dw = _data.dw * (1.0 - f_dw) + f_dw * (_data.w - w_prev) / dt;

  // UART_printStr("ticks[0]: ");
  // UART_print(ticks[0]);
  // UART_printStr(" q: ");
  // UART_printDiv(_data.q);
  // UART_printStr(" w: ");
  // UART_printDiv(_data.w);
  // UART_printStr(" dw: ");
  // UART_printDiv(_data.dw);
  // UART_printStr(" deltaW: ");
  // UART_printDiv((float)(_data.q - q_prev));
  // UART_printStr(" dtm: ");
  // UART_printDivLn(dt);

  q_prev = _data.q;
  w_prev = _data.w;
  // _data.q_deg = _data.q * 180 / 3.1415;
  t_prev = t;
  _dt = dt;
}

MotorData MotorDriver::getData()
{
  return _data;
}

void MotorDriver::setupKF()
{
  Eigen::Matrix3f C_lqe = Eigen::Matrix3f::Zero();
  C_lqe << 1, 0, 0,
      0, 1, 0,
      0, 0, 0;

  Eigen::Matrix3f At = Eigen::Matrix3f::Zero();
  Eigen::Matrix<float, 3, 2> Bt = Eigen::Matrix<float, 3, 2>::Zero();

  //maxon
  // dt = 0.01
  // _kf.Kf << 0.85107, -3.974e-09, 0, -1.987, 0.093019, 0, -0.0087006, 6.3251e-10, 0;
  // At << 0.97708, -7.7818e-20, 2.2057, -4.0585, 0.99765, 0.92537, -0.017769, 1.3583e-21, 0.92537;
  // Bt << 1.167, 0.010592, -2.4201, 2.0905, -0.010592, 0.0091529;

  //chi dt = 0.01 dist 1e2 1e3 noize 1e1 1e7
  // _kf.Kf << 0.9109, -4.125e-06, 0, -4.125, 0.080771, 0, -0.017152, 1.8013e-07, 0;

  //chi dt = 0.01 dist 1e2 1e3 noize 1e2 1e7
  // _kf.Kf << 0.59136, -2.804e-05, 0, -2.804, 0.080899, 0, -0.012237, 6.9614e-07, 0;

  //second iteration of parameters estimation dt = 0.01 dist 1e2 1e3 noize 1e1 1e7
  // _kf.Kf << 0.91266, -7.0529e-07, 0, -0.70529, 0.068304, 0, -0.0005795, -2.151e-08, 0;

  // from calculator, dt Q R same
  // _kf.Kf << 0.90575, -1.8017e-06, 0, -1.8017, 0.031967, 0, 0.0003048, -3.0667e-08, 0;

  // new wires
  _kf.Kf << 0.01663, -4.712e-05, 0, -0.4712, 0.020995, 0, -0.0011822, 3.3116e-06, 0;

  //dt 0.01
  // At << 0.93562, 8.6029e-20, 0.65758, -6.0398, 0.98288, -260.149, -0.023036, -1.9488e-21, 0.0081424;
  // Bt << 8.835, 0.16281, -42.957, 20.9421, -0.16281, 0.079869;

  //second iteration of parameters estimation dt = 0.01 dist 1e2 1e3 noize 1e1 1e7
  // At << 0.95854, 0, 10.4489, -1.3252, 0.96533, -75.0243, -0.0012254, 0, 0.93068;
  // Bt << 58.6885, 0.029038, -40.6313, 5.7045, -0.037343, 0.0052743;

  // from calculator, dt Q R same
  // At << 0.85561, -2.833e-18, 39.4323, -2.9563, 0.87118, -123.5214, -0.00074141, 3.7163e-21, 0.96944;
  // Bt << 98.5242, 0.060488, -165.1993, 11.7621, -0.040445, 0.002945;

  // new wires dt same dist 5 1, noize 1 1e4
  At << 0.69771, 6.3233e-20, 0.10871, -21.8931, 0.95896, -437.2083, -0.050891, -3.8191e-21, -0.0079294;
  Bt << 35.3874, 1.1475, -1094.6944, 98.8297, -2.4759, 0.22944;

  _kf.A_hat = At - _kf.Kf * C_lqe;
  _kf.B_hat.block<3, 2>(0, 0) = Bt;
  _kf.B_hat.block<3, 3>(0, 2) = _kf.Kf;

  _cmd.M_ext = -12.3e-3;
}

void MotorDriver::runKalmanFilter()
{
  static float Mext_prev = 0;
  static float k_m = 0.004;

  if (isnan(_cmd.M_ext))
  {
    _cmd.M_ext = 0.0;
  }

  // _kf.u(0) = _cmd.M_ext;
  _kf.u(0) = 0.0;
  _kf.u(1) = _cmd.u - _cmd.u / 6.0 * 0.31;
  _kf.u(2) = _data.w;
  _kf.u(3) = _data.dw;
  _kf.u(4) = 0.0;

  _kf.x = _kf.A_hat * _kf.x + _kf.B_hat * _kf.u;
  // _kf.x(0) = _data.w;

  // _cmd.M_ext = _param.J * _kf.x(1) + _param.Lam * _kf.x(0) - _param.Km * _kf.x(2);
  // _cmd.M_ext = -abs(_cmd.M_ext);
  _cmd.M_ext = _param.Lam * _data.w - _param.Km / _param.R * (_kf.u(1) - _param.Kw * _data.w);

  // _cmd.M_ext = Mext_prev * (1.0 - k_m) + k_m * _cmd.M_ext;
  _data.eM_ext = _cmd.M_ext;
  Mext_prev = _cmd.M_ext;

  _data.ew = _kf.x(0);
  _data.edw = _kf.x(1);

  if (abs(_kf.x(2)) < 1e-5)
  {
    _data.eI = 0;
  }
  else
  {
    // _data.eI = _kf.x(2) + _kf.x(2) / abs(_kf.x(2)) * ((abs(_kf.x(2)) - 0.045) / (0.24 - 0.045)) * (1.25 - 0.24); //mA
    // _data.eI = (_kf.u(1) - _param.Kw * _data.w) / _param.R;
    _data.eI = _kf.x(2); //mA
  }
  // _data.eI *= 1000.0;                                      //A
  // _data.M = _kf.x(2) * _param.Km;
  _data.M = _data.eI * _param.Km;

  _kf.Mext_plot = _cmd.M_ext * 1000; //mN*m
}

Eigen::Vector3f MotorDriver::getKalmanState()
{
  return _kf.x;
}