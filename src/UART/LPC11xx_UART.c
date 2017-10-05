/*
 * UART.c
 *
 *  Created on: 2016/02/14
 *      Author: evaota
 */
#include "LPC11xx_UART.h"

static const uint32_t baudtable[]={
		300,600,1200,2400,4800,9600,19200,38400,57600,115200
};

void UART_Init(UART_Init_GPIOTypedef UART_Init_GPIO, BAUDRATE_u baudrate)
{
	uint32_t DL;

	UART_Init_GPIO();

	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<12);
	LPC_SYSCON->UARTCLKDIV |= (1<<0);

	LPC_UART->LCR = 0x00000083;//Enable divisor latch access and Word length 8bit
	DL = (SystemCoreClock*LPC_SYSCON->SYSAHBCLKDIV)/(16 * baudtable[(uint8_t)baudrate]* LPC_SYSCON->UARTCLKDIV);
	LPC_UART->DLL = DL%256;
	LPC_UART->DLM = DL/256;
	LPC_UART->LCR  &= ~(1<<7);//Disable divisor latch access
	LPC_UART->FCR = 0x07;
}

void UART_Transmit(int8_t c)
{
	while(!((LPC_UART->LSR>>5)&0x01));
	LPC_UART->THR = c;
}

void UART_StringTransmit(char *s)
{
	while(*s!='\0')
	{
		UART_Transmit(*s);
		s++;
	}
}
int8_t UART_Receive(void)
{
	while(!(LPC_UART->LSR&0x01));
	return LPC_UART->RBR;
}

