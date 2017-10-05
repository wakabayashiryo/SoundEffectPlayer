/*
 * UART.h
 *
 *  Created on: 2016/02/14
 *      Author: evaota
 */

#ifndef LPC11XX_UART_H_
#define LPC11XX_UART_H_

#include "LPC11xx.h"
#include <stdint.h>

typedef enum
{
    BAUD_300    = 0,
	BAUD_600    = 1,
    BAUD_1200   = 2,
    BAUD_2400   = 3,
	BAUD_4800   = 4,
	BAUD_9600   = 5,
    BAUD_19_2K  = 6,
	BAUD_38_4k  = 7,
    BAUD_57_6K  = 8,
    BAUD_115_2K = 9,
}BAUDRATE_u;


typedef void (*UART_Init_GPIOTypedef)(void);

void UART_Init(UART_Init_GPIOTypedef UART_Init_GPIO, BAUDRATE_u baudrate);
void UART_Transmit(int8_t c);
void UART_StringTransmit(char *s);
int8_t UART_Receive(void);
#endif /* LPC11XX_UART_H_ */
