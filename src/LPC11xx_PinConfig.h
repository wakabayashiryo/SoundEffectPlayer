/*
 * LPC11xx_PinConfig.h
 *
 *  Created on: 2017/09/26
 *      Author: evaota
 */

#ifndef LPC11XX_PINCONFIG_H_
#define LPC11XX_PINCONFIG_H_

#include "main.h"

typedef struct
{
  uint32_t PIO0:1;
  uint32_t PIO1:1;
  uint32_t PIO2:1;
  uint32_t PIO3:1;
  uint32_t PIO4:1;
  uint32_t PIO5:1;
  uint32_t PIO6:1;
  uint32_t PIO7:1;
  uint32_t PIO8:1;
  uint32_t PIO9:1;
  uint32_t PIO10:1;
  uint32_t PIO11:1;
}GPIOx_Bits;

#define LPC_GPIO0_Bits ((volatile GPIOx_Bits *) (&LPC_GPIO0->DATA))
#define LPC_GPIO1_Bits ((volatile GPIOx_Bits *) (&LPC_GPIO1->DATA))
#define LPC_GPIO2_Bits ((volatile GPIOx_Bits *) (&LPC_GPIO2->DATA))


#endif /* LPC11XX_PINCONFIG_H_ */
