#pragma once

#include "gpio.h"
#include "i2c.h"
#include "main.h"
#include "stdint.h"
#include "stm32f401re/splitmind_f401_hal_lib.h"
#include "stm32f401re/usart_debug/usart_debug.h"
#include "tim.h"
#include "usart.h"

#define INA219_ADDRESS 0x82
#define CONFIGURATION 0x00
#define CURRENT 0x04
#define SHUNT_VOLTAGE 0x01

extern uint8_t config[2];
extern uint16_t configuration;
extern uint8_t reg;
extern bool is_new_data;
extern int16_t current;
extern int16_t voltage;
extern uint8_t data[3];

extern void changeConfig(void);
extern void getData(void);
extern void printData(void);
void readCurrent(void);
void readVoltage(void);

// void readCurrent(void);

//interruption after tx
extern void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c);
//recieve data interrupt
extern void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c);