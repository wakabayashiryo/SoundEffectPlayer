/*
 * SPI.h
 *
 *  Created on: 2016/02/24
 *      Author: evaota
 */

#ifndef LPC11XX_SPI_H_
#define LPC11XX_SPI_H_

#include "LPC11xx.h"
#include <stdint.h>

typedef enum
{
	SSPMASTER,
	SSPSLAVE
}SPI_MODE;

typedef void(*SPI_Init_GPIOTypedef)(void);

void SPI_Init(SPI_Init_GPIOTypedef SPI_Init_GPIO,uint8_t ClockDivid,SPI_MODE mode);//Initializing SSP module
void SPI_Transmit(uint8_t data);//Transmit one byte data
uint8_t SPI_Receive(void);//Receive one byte data

#endif /* LPC11XX_SPI_H_ */
