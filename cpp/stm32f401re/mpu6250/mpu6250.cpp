#include "mpu6250.h"

bool is_new_data = false;
uint8_t data_frame[14] = {0};
uint8_t data_send[14] = {0};
uint8_t data_frame_raw[14] = {0};
uint8_t recv_size = 2;
uint8_t send_size = 2;

// interruption after tx
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
  // start waiting for recieve interrupt
  HAL_I2C_Master_Receive_IT(&hi2c1, MPU6250_ADDRESS, data_frame_raw, recv_size);
}

// recieve data interrupt
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
  // configuration = config[0] << 8 | config[1];
  // *data_frame = *data_frame_raw;

  for (size_t i = 0; i < 14; i++)
  {
    /* code */
    data_frame[i] = data_frame_raw[i];
  }

  is_new_data = true;
}

// void getData(void)
// {
//   HAL_I2C_Master_Transmit_IT(&hi2c1, MPU6250_ADDRESS, &reg, 1);
// }

// void printData(void)
// {
//   if (is_new_data)
//   {
//     UART_printStr("Data: ");

//     // if (configuration & (1 << 15))
//     // {

//     // }

//     // UART_printLn(current / 10);
//     UART_printLn(configuration);

//     is_new_data = false;
//   }
// }

// void changeConfig(void)
// {
//   data[0] = CONFIGURATION;
//   data[1] = 0x39;
//   // data[2] = 0xDF;
//   data[2] = 0xE7;
//   HAL_I2C_Master_Transmit(&hi2c1, INA219_ADDRESS, data, 3, HAL_TIMEOUT);
// }

MPU6250::MPU6250(uint8_t device_address)
{
  _device_address = device_address;

  R_gyro = Eigen::Matrix<float, 2, 3>::Zero();
  data.dOri = Eigen::Vector2f::Zero();

  _GyX_offset = -25;
  _GyY_offset = 2;
  _GyZ_offset = 17;
}

uint8_t MPU6250::init()
{
  data_send[0] = 0x6B;
  data_send[1] = 0;
  send_size = 2;

  HAL_I2C_Master_Transmit(&hi2c1, MPU6250_ADDRESS, data_send, send_size, HAL_TIMEOUT);

  HAL_Delay(20);

  data_send[0] = WHO_AM_I;
  send_size = 1;
  recv_size = 1;
  HAL_I2C_Master_Transmit_IT(&hi2c1, MPU6250_ADDRESS, data_send, send_size);
  HAL_Delay(10);
  int8_t a = data_frame[0];

#ifdef INIT_INFO
  //0x70
  UART_printStr("MPU6250 WHO AM I: ");
  UART_printLn(a);
#endif

  if (a == 0x70)
  {
#ifdef INIT_INFO
    UART_printStrLn("MPU6250 INIT SUCCSESS!!!");
#endif
    // return SUCCSESS;
  }
  else
  {
#ifdef INIT_INFO
    UART_printStrLn("MPU6250 INIT FAILURE!!!");
#endif
    return ERROR_INIT;
  }

  changeSampleRate(9); //update rate of sensor register. 1000/(1 + 9)= 100 Hz
  filterAccelerometer(2);
  rangeAccelerometer(0b00011000);
  // calibrationAccelerometer(200);

  // filterGyroscope(2);
  filterGyroscope(4);
  rangeGyroscope(0b00011000);
  // rangeGyroscope(0);
  // calibrationGyroscope(200);

  return SUCCSESS;
}

void MPU6250::changeSampleRate(int8_t Koeff)
{
  data_send[0] = SMPLRT_DIV;
  data_send[1] = Koeff;

  HAL_I2C_Master_Transmit(&hi2c1, MPU6250_ADDRESS, data_send, 2, HAL_TIMEOUT);

  HAL_Delay(10);
}

void MPU6250::rangeAccelerometer(int8_t mode)
{
  data_send[0] = 28;
  data_send[1] = mode;

  HAL_I2C_Master_Transmit(&hi2c1, MPU6250_ADDRESS, data_send, 2, HAL_TIMEOUT);

  HAL_Delay(10);
}

void MPU6250::filterAccelerometer(int16_t mode)
{
  data_send[0] = 29;
  data_send[1] = mode;

  HAL_I2C_Master_Transmit(&hi2c1, MPU6250_ADDRESS, data_send, 2, HAL_TIMEOUT);

  HAL_Delay(10);
}

void MPU6250::calibrationAccelerometer(int16_t number_of_tacts)
{
  UART_printStrLn("Calibration of accelerometer... ");

  for (int16_t i = 0; i < number_of_tacts; i++)
  {
    data_send[0] = 0x3B;
    send_size = 1;
    recv_size = 6;
    HAL_I2C_Master_Transmit_IT(&hi2c1, MPU6250_ADDRESS, data_send, send_size);
    HAL_Delay(10);

    _AcX_offset += (int16_t)(data_frame[0] << 8 | data_frame[1]); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    _AcY_offset += (int16_t)(data_frame[2] << 8 | data_frame[3]); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    _AcZ_offset += (int16_t)(data_frame[4] << 8 | data_frame[5]); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  }

  _AcX_offset = _AcX_offset / number_of_tacts;
  _AcY_offset = _AcY_offset / number_of_tacts;
  _AcZ_offset = _AcZ_offset / number_of_tacts;

  UART_printStrLn("Calibration of accelerometer complete: ");
  UART_printStr("AcX_offset = ");
  UART_print(_AcX_offset);
  UART_printStr(" | AcY_offset = ");
  UART_print(_AcY_offset);
  UART_printStr(" | AcZ_offset = ");
  UART_printLn(_AcZ_offset);
}

void MPU6250::rangeGyroscope(int8_t mode)
{
  data_send[0] = 27;
  data_send[1] = mode;

  HAL_I2C_Master_Transmit(&hi2c1, MPU6250_ADDRESS, data_send, 2, HAL_TIMEOUT);

  HAL_Delay(10);
}

void MPU6250::filterGyroscope(int8_t mode)
{
  data_send[0] = 26;
  data_send[1] = mode;

  HAL_I2C_Master_Transmit(&hi2c1, MPU6250_ADDRESS, data_send, 2, HAL_TIMEOUT);

  HAL_Delay(10);
}

void MPU6250::calibrationGyroscope(int16_t number_of_tacts)
{
  UART_printStrLn("Calibration of gyroscope... ");

  for (int16_t i = 0; i < number_of_tacts; i++)
  {
    data_send[0] = 0x43;
    send_size = 1;
    recv_size = 6;
    HAL_I2C_Master_Transmit_IT(&hi2c1, MPU6250_ADDRESS, data_send, send_size);
    HAL_Delay(10);

    _GyX_offset += (int16_t)(data_frame[0] << 8 | data_frame[1]); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    _GyY_offset += (int16_t)(data_frame[2] << 8 | data_frame[3]); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    _GyZ_offset += (int16_t)(data_frame[4] << 8 | data_frame[5]); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  }

  _GyX_offset = _GyX_offset / number_of_tacts;
  _GyY_offset = _GyY_offset / number_of_tacts;
  _GyZ_offset = _GyZ_offset / number_of_tacts;

  UART_printStrLn("Calibration of gyroscope complete: ");
  UART_printStr("GyX_offset = ");
  UART_print(_GyX_offset);
  UART_printStr(" | GyY_offset = ");
  UART_print(_GyY_offset);
  UART_printStr(" | GyZ_offset = ");
  UART_printLn(_GyZ_offset);
}

void MPU6250::getData()
{
  data_send[0] = 0x3B;
  send_size = 1;
  recv_size = 14;
  HAL_I2C_Master_Transmit_IT(&hi2c1, MPU6250_ADDRESS, data_send, send_size);

  data.RawAcX = (int16_t)(data_frame[0] << 8 | data_frame[1]); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  data.RawAcY = (int16_t)(data_frame[2] << 8 | data_frame[3]); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  data.RawAcZ = (int16_t)(data_frame[4] << 8 | data_frame[5]); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  data.Tmp = data_frame[6] << 8 | data_frame[7]; // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

  data.RawGyX = (int16_t)(data_frame[8] << 8 | data_frame[9]);   // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  data.RawGyY = (int16_t)(data_frame[10] << 8 | data_frame[11]); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  data.RawGyZ = (int16_t)(data_frame[12] << 8 | data_frame[13]); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  data.AcX = data.RawAcX - _AcX_offset;
  data.AcY = data.RawAcY - _AcY_offset;
  data.AcZ = data.RawAcZ - _AcZ_offset;
  data.GyX = data.RawGyX - _GyX_offset;
  data.GyY = data.RawGyY - _GyY_offset;
  data.GyZ = data.RawGyZ - _GyZ_offset;

  data.AcX *= 16.0 * 9.81 / 32768.0;
  data.AcY *= 16.0 * 9.81 / 32768.0;
  data.AcZ *= 16.0 * 9.81 / 32768.0;

  data.GyX *= 2000.0 / 32768.0;
  data.GyY *= 2000.0 / 32768.0;
  data.GyZ *= 2000.0 / 32768.0;

  data.AcX *= -1.0;
  data.AcY *= -1.0;
  data.AcZ *= -1.0;

  float tmp = data.AcY;
  data.AcY = data.AcX;
  data.AcX = -tmp;

  tmp = data.GyY;
  data.GyY = data.GyX;
  data.GyX = -tmp;

  // printData();
}

void MPU6250::printData()
{
#ifdef SHOW_DATA
  UART_printStr("AcX = ");
  UART_print(data.AcX);
  UART_printStr(" | AcY = ");
  UART_print(data.AcY);
  UART_printStr(" | AcZ = ");
  UART_print(data.AcZ);
  UART_printStr(" | GyX = ");
  UART_print(data.GyX);
  UART_printStr(" | GyY = ");
  UART_print(data.GyY);
  UART_printStr(" | GyZ = ");
  UART_printLn(data.GyZ);
#endif // SHOW_DATA
}

void MPU6250::printRawData()
{
#ifdef SHOW_DATA
  UART_printStr("AcX = ");
  UART_printDiv(data.RawAcX);
  UART_printStr(" | AcY = ");
  UART_printDiv(data.RawAcY);
  UART_printStr(" | AcZ = ");
  UART_printDiv(data.RawAcZ);
  UART_printStr(" | GyX = ");
  UART_printDiv(data.RawGyX);
  UART_printStr(" | GyY = ");
  UART_printDiv(data.RawGyY);
  UART_printStr(" | GyZ = ");
  UART_printDivLn(data.GyZ);
#endif // SHOW_DATA
}

void MPU6250::plotData()
{
  UART_printStr("AcX = ");
  UART_printDiv(data.AcX);
  UART_printStr(", AcY = ");
  UART_printDiv(data.AcY);
  UART_printStr(", AcZ = ");
  UART_printDiv(data.AcZ);
  UART_printStr(", GyX = ");
  UART_printDiv(data.GyX);
  UART_printStr(", GyY = ");
  UART_printDiv(data.GyY);
  UART_printStr(", GyZ = ");
  UART_printDivLn(data.GyZ);
  UART_printStr("\r");
}

void MPU6250::lowPassFilter()
{
  float f_acc = 0.9;
  float f_gyro = 0.9;
  static Eigen::Vector3f gyro_prev(0, 0, 0);
  static Eigen::Vector3f acc_prev(0, 0, 0);

  data.AcX = acc_prev(0) * (1.0 - f_acc) + f_acc * data.AcX;
  data.AcY = acc_prev(1) * (1.0 - f_acc) + f_acc * data.AcY;
  data.AcZ = acc_prev(2) * (1.0 - f_acc) + f_acc * data.AcZ;

  data.GyX = gyro_prev(0) * (1.0 - f_gyro) + f_gyro * data.GyX;
  data.GyY = gyro_prev(1) * (1.0 - f_gyro) + f_gyro * data.GyY;
  data.GyZ = gyro_prev(2) * (1.0 - f_gyro) + f_gyro * data.GyZ;

  acc_prev(0) = data.AcX;
  acc_prev(1) = data.AcY;
  acc_prev(2) = data.AcZ;

  gyro_prev(0) = data.GyX;
  gyro_prev(1) = data.GyY;
  gyro_prev(2) = data.GyZ;
}

void MPU6250::complimentaryFilter()
{
  data.acc_pitch = atan2(data.AcZ, data.AcX) + 3.1415 / 2.0;
  data.acc_roll = atan2(data.AcZ, data.AcY) + 3.1415 / 2.0;

  R_gyro << 1.0, sin(data.roll) * tan(data.pitch), cos(data.roll) * tan(data.pitch), 0.0, cos(data.roll), -sin(data.pitch);

  unsigned long t = getStMcs();

  float dt = (float)(t - data.t_prev) / 1e6;
  data.t_prev = t;

  Eigen::Vector3f gyro(data.GyX, data.GyY, data.GyZ);
  gyro = gyro * 3.1415 / 180.0;
  data.dOri = R_gyro * gyro;

  float f = 0.03;

  data.roll = data.acc_roll * f + (1.0 - f) * (data.roll + dt * data.dOri(0));
  data.pitch = data.acc_pitch * f + (1.0 - f) * (data.pitch + dt * data.dOri(1));

  // UART_printStr("roll: ");
  // UART_printDiv(data.roll * 180.0 / 3.1415);
  // // UART_printStr(", acc_roll: ");
  // // UART_printDiv(data.acc_roll * 180.0 / 3.1415);
  // UART_printStr(", pitch: ");
  // UART_printDiv(data.pitch * 180.0 / 3.1415);
  // // UART_printStr(", dpitch: ");
  // // UART_printDiv(data.dOri(1) * 180.0 / 3.1415);
  // // UART_printStr(", GyY: ");
  // // UART_printDiv(data.GyY * 180.0 / 3.1415);
  // // UART_printStr(", acc_pitch: ");
  // // UART_printDiv(data.acc_pitch * 180.0 / 3.1415);
  // UART_printStrLn("\r");
}
