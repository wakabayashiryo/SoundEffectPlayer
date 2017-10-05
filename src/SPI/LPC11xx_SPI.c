/*
 * SPI.c
 *
 *  Created on: 2016/02/24
 *      Author: evaota
 */
#include "LPC11xx_SPI.h"

void SPI_Init(SPI_Init_GPIOTypedef SPI_Init_GPIO,uint8_t ClockDivid,SPI_MODE mode)
{
	SPI_Init_GPIO();

	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<11);//supplied system clock to SSP module
	LPC_SYSCON->SSP0CLKDIV = 0x01;//set system clock divider
	LPC_SYSCON->PRESETCTRL |= (1<<0);//de-assert SSP module

	LPC_SSP0->CR0 |= (7<<0);//transmit data size is 8bit
	LPC_SSP0->CR0 &= ~(1<<6);//CPOL is zero
	LPC_SSP0->CR0 &= ~(1<<7);//CPHA is zero
	LPC_SSP0->CR0 |= (1<<8);//serial clock rate is 1:1 24MHz

	if(ClockDivid<2)ClockDivid = 2;

	switch(mode)
	{
		case SSPMASTER:
			LPC_SSP0->CR1 &= ~(1<<2);//select SSP master mode
			LPC_SSP0->CPSR = ClockDivid;//clock divider is 1:2
		break;
		case SSPSLAVE:
			LPC_SSP0->CR1 |= (1<<2);//select SSP slave mode
		break;
	}

	LPC_SSP0->CR1 |= (1<<1);//enable spi controller
}
void SPI_Transmit(uint8_t data)
{
	while ( (LPC_SSP0->SR & ((1<<1)|(1<<4))) != (1<<1) );//wait not busy and not transmit FIFO full
	LPC_SSP0->DR = data;//transmit data
	while ( (LPC_SSP0->SR & ((1<<2)|(1<<4))) != (1<<2) );//wait not busy and not receive FIFO empty
	LPC_SSP0->DR;//receive FIFO
}

uint8_t SPI_Receive(void)
{
	LPC_SSP0->DR = 0xff;//transmit dummy data
	while ( (LPC_SSP0->SR & ((1<<4)|(1<<2))) != (1<<2) );//wait not busy and not receive FIFO empty
	return LPC_SSP0->DR;//read receive FIFO
}

