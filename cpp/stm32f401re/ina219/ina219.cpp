#include "ina219.h"

uint8_t config[2];
uint8_t reg = CONFIGURATION;
uint16_t configuration = 0;
int16_t current = 0;
int16_t voltage = 0;
bool is_new_data = false;
uint8_t data[3] = {0, 0, 0};

// interruption after tx
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
  // start waiting for recieve interrupt
  HAL_I2C_Master_Receive_IT(&hi2c1, INA219_ADDRESS, config, 2);
}

// recieve data interrupt
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
  // I2C data ready!
  // UART_printStr("Configuration: ");

  // uint16_t configure = 0;
  // configure = config[0] << 8 | config[1];
  // UART_printLn(configure);

  configuration = config[0] << 8 | config[1];
  current = config[0] << 8 | config[1];
  voltage = config[0] << 8 | config[1];

  is_new_data = true;
}

void getData(void)
{
  HAL_I2C_Master_Transmit_IT(&hi2c1, INA219_ADDRESS, &reg, 1);
}

void readVoltage(void)
{
  reg = 2;
  HAL_I2C_Master_Transmit_IT(&hi2c1, INA219_ADDRESS, &reg, 1);
}

void readCurrent(void)
{
  reg = SHUNT_VOLTAGE;
  HAL_I2C_Master_Transmit_IT(&hi2c1, INA219_ADDRESS, &reg, 1);
}

void printData(void)
{
  if (is_new_data)
  {
    UART_printStr("Data: ");

    // if (configuration & (1 << 15))
    // {

    // }

    // UART_printLn(current / 10);
    UART_printLn(configuration);

    is_new_data = false;
  }
}

void changeConfig(void)
{
  data[0] = CONFIGURATION;

  data[1] = 0x39;
  // data[2] = 0xDF;
  data[2] = 0xE7;

  // data[1] = 0x39;
  // data[2] = 0xFF;
  HAL_I2C_Master_Transmit(&hi2c1, INA219_ADDRESS, data, 3, HAL_TIMEOUT);
}
