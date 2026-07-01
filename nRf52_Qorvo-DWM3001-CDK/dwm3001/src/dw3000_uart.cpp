/*
 * UART.c
 *
 * Created: 9/10/2021 12:32:14 PM
 *  Author: Emin Eminof
 */ 
#include "dw3000_uart.h"


// Use the nRF UART and allow pin override for external headers.
#ifndef UART_PORT
#define UART_PORT Serial
#endif

#ifndef UART_PIN_RX
#if defined(DWM3001_CDK)
#define UART_PIN_RX 15
#define UART_PIN_TX 14
#else
#define UART_PIN_RX PIN_SERIAL_RX
#define UART_PIN_TX PIN_SERIAL_TX
#endif
#endif

void UART_init(void)
{
  UART_PORT.setPins(UART_PIN_RX, UART_PIN_TX);
  UART_PORT.begin(115200);
  UART_PORT.println("UART ready");
}

void UART_putc(char data)
{
  UART_PORT.print(data);
}

void UART_puts(const char* s)
{
  UART_PORT.print(s);
}

void test_run_info(unsigned char * s)
{
    UART_puts((char *)s);
    UART_puts("\r\n");
}
