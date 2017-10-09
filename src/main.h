#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include "PWM/LPC11xx_PWM.h"
#include "SPI/LPC11xx_SPI.h"
#include "Timer16_32/LPC11xx_Timer16_32.h"
#include "LPC11xx_PinConfig.h"

#define LED_GREEN 		LPC_GPIO1_Bits->PIO9
#define LED_RED			LPC_GPIO1_Bits->PIO8

#define PIR_STAT()			LPC_GPIO2_Bits->PIO4

void SPI_Init_GPIO(void);

#endif
