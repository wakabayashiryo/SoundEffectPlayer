/*
 * PWM.c
 *
 *  Created on: 2016/02/15
 *      Author: evaota
 */
#include "LPC11xx_PWM.h"

//frequency range is 60Hz~49kHz

void PWM16B0_Init(LPC_PWM_Config *pwm_config)//pwm cycle is generated by MR3.LPC1114 don't exist CT16B0_MAT3 pin,
{
	pwm_config->timer_conf.TMRx = LPC_TMR16B0;
	pwm_config->timer_conf.SetNumMRx = 3;
	pwm_config->timer_conf.MR3Value = 1023-1;//set PWM period,minimum cycle is 20us.
	pwm_config->timer_conf.MaskClearCT = 1;//Reset CT By MR3
	pwm_config->timer_conf.MaskRaiseHandle = 0;
	pwm_config->timer_conf.Prescaler = pwm_config->Prescaler_pwm;//user count
	pwm_config->timer_conf.MR0Value = 0x0000;//clear output matching register
	pwm_config->timer_conf.MR1Value = 0x0000;//clear output matching register

	Timer16_32_Init(&pwm_config->timer_conf);

	LPC_TMR16B0->PWMC |= (pwm_config->Mask_outpin<<0);//MAT0 IS PWM MODE
}
void PWM16B1_Init(LPC_PWM_Config *pwm_config)//pwm cycle is generated by MR3.MAT1~MAT3 can't use PWM output,
{
	pwm_config->timer_conf.TMRx = LPC_TMR16B1;
	pwm_config->timer_conf.SetNumMRx = 3;
	pwm_config->timer_conf.MR3Value = 1023-1;//set PWM period,minimum cycle is 20us.
	pwm_config->timer_conf.MaskClearCT = 1;//Reset CT By MR3
	pwm_config->timer_conf.MaskRaiseHandle = 0;
	pwm_config->timer_conf.Prescaler = pwm_config->Prescaler_pwm;//user count

	Timer16_32_Init(&pwm_config->timer_conf);

	LPC_TMR16B1->PWMC |= (pwm_config->Mask_outpin<<0);//MAT0 IS PWM MODE
}
void PWM32B0_Init(LPC_PWM_Config *pwm_config)//pwm cycle is generated by MR1.MAT1 can't use PWM output,because MAT1 is used UART with TX pin
{
	pwm_config->timer_conf.TMRx = LPC_TMR32B0;
	pwm_config->timer_conf.SetNumMRx = 1;
	pwm_config->timer_conf.MR3Value = 1023-1;//set PWM period,minimum cycle is 20us.
	pwm_config->timer_conf.MaskClearCT = 1;//Reset CT By MR3
	pwm_config->timer_conf.MaskRaiseHandle = 0;
	pwm_config->timer_conf.Prescaler = pwm_config->Prescaler_pwm;//user count
	pwm_config->timer_conf.MR0Value = 0x00000000;//clear output matching register
	pwm_config->timer_conf.MR2Value = 0x00000000;//clear output matching register
	pwm_config->timer_conf.MR3Value = 0x00000000;//clear output matching register

	Timer16_32_Init(&pwm_config->timer_conf);

	LPC_TMR32B0->PWMC |= (pwm_config->Mask_outpin<<0);//MAT0 IS PWM MODE
}
void PWM32B1_Init(LPC_PWM_Config *pwm_config)//pwm cycle is generated by MR2.MAT2 can't use PWM output.
{
	pwm_config->timer_conf.TMRx = LPC_TMR32B1;
	pwm_config->timer_conf.SetNumMRx = 2;
	pwm_config->timer_conf.MR2Value = 1023-1;//set PWM period,minimum cycle is 20us.
	pwm_config->timer_conf.MaskClearCT = 1;//Reset CT By MR2
	pwm_config->timer_conf.MaskRaiseHandle = 0;
	pwm_config->timer_conf.Prescaler = pwm_config->Prescaler_pwm;//user count
	pwm_config->timer_conf.MR0Value = 0;//clear output matching register
	pwm_config->timer_conf.MR1Value = 0;//clear output matching register
	pwm_config->timer_conf.MR3Value = 0;//clear output matching register

	Timer16_32_Init(&pwm_config->timer_conf);

	LPC_TMR32B1->PWMC |= (pwm_config->Mask_outpin<<0);//MAT3 IS PWM MODE
}

void PWM_Set_duty(LPC_TMR_TypeDef *TMRx,uint32_t duty)
{
	TMRx->MR0 = duty;
}

