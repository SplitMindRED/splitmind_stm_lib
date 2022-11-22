#pragma once

#include "gpio.h"
#include "main.h"
#include "math.h"
#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#include "usart.h"
#include "string"
#include "sstream"
#include "iomanip"

#define UART_DEBUG

#define UART1 &huart1
#define UART2 &huart2
#define UART6 &huart6

#define MAX_DELAY 15
#define OK 1
#define ERROR -1

// UART for debug
#ifdef UART_DEBUG
#define UDBG UART2
#endif

/*
 ******************************************************************************
 * UART print stuff
 ******************************************************************************
 */
#ifdef UART_DEBUG
void UART_sendByte(char byte);
void UART_print(long data);
void UART_printStr(char *string);
void UART_printDiv(double data);
void UART_printLn(long data);
void UART_printStrLn(char *string);
void UART_printDivLn(double data);
#endif
/*
 ******************************************************************************
 */
