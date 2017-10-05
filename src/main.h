#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include "PWM/LPC11xx_PWM.h"
#include "SPI/LPC11xx_SPI.h"
#include "Timer16_32/LPC11xx_Timer16_32.h"
#include "UART/LPC11xx_UART.h"
#include "LPC11xx_PinConfig.h"

#define LED_BLUE 		LPC_GPIO0_Bits->PIO4
#define LED_GREEN 	LPC_GPIO1_Bits->PIO4
#define LED_RED	 	LPC_GPIO0_Bits->PIO5

#define SW_RIGHT()		LPC_GPIO0_Bits->PIO1
#define SW_CENTER() 	LPC_GPIO1_Bits->PIO9
#define SW_LEFT()			LPC_GPIO1_Bits->PIO8

#define SW_HOLD() 		LPC_GPIO0_Bits->PIO3

void SPI_Init_GPIO(void);

#endif
