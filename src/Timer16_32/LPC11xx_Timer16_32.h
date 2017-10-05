/*
 * Timer16_32.h
 *
 *  Created on: 2016/02/13
 *      Author: evaota
 */

#ifndef TIMER16_32_TIMER16_32_H_
#define TIMER16_32_TIMER16_32_H_

#include "LPC11xx.h"
#include <stdint.h>

typedef struct
{
	LPC_TMR_TypeDef *TMRx;

	uint32_t MR0Value;
	uint32_t MR1Value;
	uint32_t MR2Value;
	uint32_t MR3Value;

	uint32_t MaskRaiseHandle;
	uint32_t MaskClearCT;

	uint32_t SetNumMRx;
	uint32_t Prescaler;

}LPC_TMRx_Config;

typedef enum
{
	NONE,
	MATCHHIGH,
	MATCHLOW,
	MATCHTGL
}EMR_MODE;


#define EnableTMR16B0IRQ() NVIC_EnableIRQ(TIMER_16_0_IRQn)
#define EnableTMR16B1IRQ() NVIC_EnableIRQ(TIMER_16_1_IRQn)
#define EnableTMR32B0IRQ() NVIC_EnableIRQ(TIMER_32_0_IRQn)
#define EnableTMR32B1IRQ() NVIC_EnableIRQ(TIMER_32_1_IRQn)

void Timer16_32_Init(LPC_TMRx_Config *config);
void Timer16_32_Clear_IR(LPC_TMRx_Config *config);

#endif /* TIMER16_32_TIMER16_32_H_ */
