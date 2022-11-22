#include "splitmind_f401_hal_lib.h"

volatile unsigned long st_ms = 0;

//   HAL_OK       = 0x00U,
//   HAL_ERROR    = 0x01U,
//   HAL_BUSY     = 0x02U,
//   HAL_TIMEOUT  = 0x03U

void turnLed(bool status)
{
  if (status == 1)
  {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  }
}

uint32_t getStMcs()
{
  return st_ms * 1000 + __HAL_TIM_GET_COUNTER(&htim11);
  // return __HAL_TIM_GET_COUNTER(&htim11);
}

int8_t getSign(float var)
{
  if (var < 0)
  {
    return -1;
  }
  else
  {
    return 1;
  }
}
