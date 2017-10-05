/*
 * Timer16_32.c

 *
 *  Created on: 2016/02/13
 *      Author: evaota
 */
#include "LPC11xx_Timer16_32.h"

void Timer16_32_SetExternalOut(LPC_TMR_TypeDef *TMRx,uint8_t MRx,EMR_MODE mode)
{
		TMRx->EMR |= (((uint32_t)mode<<MRx)<<4);
}

void Timer16_32_Init(LPC_TMRx_Config *config)
{
	//supplied system clock each TMRx
	if(config->TMRx==LPC_TMR16B0)
		LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);
	if(config->TMRx==LPC_TMR16B1)
		LPC_SYSCON->SYSAHBCLKCTRL |= (1<<8);
	if(config->TMRx==LPC_TMR32B0)
		LPC_SYSCON->SYSAHBCLKCTRL |= (1<<9);
	if(config->TMRx==LPC_TMR32B1)
		LPC_SYSCON->SYSAHBCLKCTRL |= (1<<10);

	config->TMRx->MR0 = config->MR0Value;
	config->TMRx->MR1 = config->MR1Value;
	config->TMRx->MR2 = config->MR2Value;
	config->TMRx->MR3 = config->MR3Value;

	config->TMRx->MCR |= (config->MaskRaiseHandle<<(config->SetNumMRx*3));	  //Raise handle
	config->TMRx->MCR |= (config->MaskClearCT       <<(config->SetNumMRx*3+1));//Clear CT

	config->TMRx->PR = config->Prescaler;			//Set prescaler


	config->TMRx->CTCR = 0;
	config->TMRx->TCR = 1;//ENABLE COUNTER
}

void Timer16_32_Clear_IR(LPC_TMRx_Config *config)
{
	config->TMRx->IR |= (1<<config->SetNumMRx);
}
