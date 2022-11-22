/***********************************************
 *	SplitMind Library for STM32F401RE using HAL
 *	Version 0.2
 *
 *  Functions for low level initialization and control
 *
 *	WRITING BITs
 *	a |= 1 << 7;					//set 1 in 7th bit
 *	a &= ~(1 << 3);				//set 0 in 3th bit
 *	a ^= 1 << 5;					//inversion of 5th bit
 *	a |= 1 << 7 | 1 << 8		   //sets 1 to 7th and 8th bits
 *
 *	READING BITs
 *	if( a & (1<<7) )				//if 7th bit in "a" var
 *  equals 1-> true a & ( 1 << 7 | 1 << 8 )	   //checks 7th and 8th bits
 *
 ************************************************/

#pragma once

#include "gpio.h"
#include "main.h"
#include "math.h"
#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "usart.h"

#define MAX_DELAY 15
#define OK 1
#define ERROR -1

extern volatile unsigned long st_ms;

typedef struct SoftTimer_ms
{
  unsigned long delay;
  unsigned long start_time;
} SoftTimer_ms;

bool checkTimer(SoftTimer_ms* timer);

int8_t getSign(float var);

void turnLed(bool status);

uint32_t getStMcs();
