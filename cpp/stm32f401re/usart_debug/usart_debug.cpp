#include "usart_debug.h"

/*
 ******************************************************************************
 * UART print stuff
 ******************************************************************************
 */

#ifdef UART_DEBUG

void UART_printNumber(unsigned long number)
{
  char string[11];
  char* pointer = &string[sizeof(string) - 1];

  *pointer = '\0';
  pointer--;

  if (number == 0)
  {
    *(pointer--) = '0';
  }
  else
  {
    while (number)
    {
      char symbol = number % 10;
      number = number / 10;

      *(pointer--) = symbol + '0';
    }
  }

  HAL_UART_Transmit(UDBG, (uint8_t*)(pointer + 1),
                    strlen((char*)(pointer + 1)), HAL_MAX_DELAY);
}

void UART_sendByte(char byte)
{
  HAL_UART_Transmit(UDBG, (uint8_t*)(&byte), 1, HAL_MAX_DELAY);
}

void UART_print(long data)
{
  if (data < 0)
  {
    UART_sendByte('-');
    data = -data;
  }

  UART_printNumber(data);
}

void UART_printStr(char* string)
{
  HAL_UART_Transmit(UDBG, (uint8_t*)string, strlen(string), HAL_MAX_DELAY);
}

void UART_printDiv(double data)
{
  if (isnan(data))
  {
    // UART_printStr("nan");
    UART_printStr("0");
    return;
  }
  else if (isinf(data))
  {
    // UART_printStr("inf");
    UART_printStr("0");
    return;
  }
  else if (data > 4294967040.0)
  {
    // UART_printStr("ovf");
    UART_printStr("0");
    return;
  }
  else if (data < -4294967040.0)
  {
    // UART_printStr("-ovf");
    UART_printStr("0");
    return;
  }

  if (data < 0.0)
  {
    UART_sendByte('-');
    data = -data;
  }

  std::string str;

  std::stringstream stream;
  stream << std::fixed << std::setprecision(4) << data;
  // stream << std::fixed << std::setprecision(2) << data;
  str = stream.str();

  long rounding = 0;
  long number_left = data;
  uint8_t number_right = 0;

  rounding = data * 1000;
  uint8_t tmp = rounding % 10;
  rounding = rounding / 10;

  if (tmp >= 5)
  {
    number_right = (long)(rounding + 1) % 100;
  }
  else
  {
    number_right = (long)rounding % 100;
  }

  // UART_print(number_left);
  // UART_sendByte('.');
  // if (rounding / 10 == 0)
  // {
  //   UART_print(0);
  // }
  // UART_print(number_right);

  char arr[str.length() + 1];

  strcpy(arr, str.c_str());
  UART_printStr(arr);
}

void UART_printLn(long data)
{
  UART_print(data);

  UART_sendByte('\n');
}

void UART_printStrLn(char* string)
{
  UART_printStr(string);

  UART_sendByte('\n');
}

void UART_printDivLn(double data)
{
  UART_printDiv(data);

  UART_sendByte('\n');
}

#endif
/*
 ******************************************************************************
 */
