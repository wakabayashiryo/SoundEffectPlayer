/*
===============================================================================
 Name        : SoundEffectPlayer.c
 Author      : $Ryo Wakabayashi
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/
#ifdef __USE_CMSIS
#include "LPC11xx.h"
#endif

#include <stdlib.h>
#include "main.h"
#include "wav/wav.h"
#include "fatfs/diskio.h"
#include "fatfs/ff.h"
#include <cr_section_macros.h>

#define BUFFERSIZE (512*2)
#define HALFSIZE (BUFFERSIZE>>1)
uint8_t Buff[BUFFERSIZE];

#define PLAY_TIME_MS 20000
uint32_t play_t;
#define PLAYCTRL(x) ((x==true)? NVIC_EnableIRQ(TIMER_16_0_IRQn):NVIC_DisableIRQ(TIMER_16_0_IRQn))

uint32_t Bri=0,Bwi=0,Bct=0;

extern void disk_timerproc (void);

void Basic_Init(void);
void Clear_Buffer(void);
void ErrorHandle(void);

LPC_TMRx_Config timer_conf;
LPC_PWM_Config pconfig;

volatile uint32_t delay;
#define LPC_delay(time) {\
		delay = time;\
		while(delay);\
}


int main(void)
{
		FATFS fs;
		FIL fil;

		FormatChunk_t FmCk;
		SizeWav_t WvSz;
		uint32_t decoy=0;

		bool play = false;
		play_t = PLAY_TIME_MS;

		Basic_Init();

		LED_RED= 1;
		LED_GREEN = 1;

		LPC_IOCON->R_PIO1_1 = 0x83;//PWM output left channel [MAT0 output mode]
		LPC_IOCON->R_PIO1_2 = 0x83;//PWM output right channel [MAT1 output mode]

		pconfig.Mask_outpin = 0x03;
		pconfig.Prescaler_pwm = 1;
		PWM32B1_Init(&pconfig);

		timer_conf.TMRx = LPC_TMR16B0;
		timer_conf.MaskClearCT = 1;
		timer_conf.MaskRaiseHandle = 1;
		timer_conf.SetNumMRx = 0;
		timer_conf.MR0Value = 1093;
		timer_conf.Prescaler = 0;
		Timer16_32_Init(&timer_conf);

		NVIC_SetPriority(TIMER_16_0_IRQn,0);
		EnableTMR16B0IRQ();

		SysTick_Config(SystemCoreClock/(1001-1));

		if(disk_initialize(0))
				ErrorHandle();

		if(f_mount(&fs,"",0)!=FR_OK)
				ErrorHandle();

		while(1)
		{
			while(PIR_STAT());

			play_t = PLAY_TIME_MS;

			if(f_open(&fil,"sound.wav",FA_READ)!=FR_OK)
				ErrorHandle();

			CheckWavFile(&fil,&FmCk,&WvSz);
//			DispWavInfo(&FmCk,&WvSz);

			if(f_read(&fil,&Buff[0],BUFFERSIZE,&decoy)!=FR_OK)
				ErrorHandle();
			Bct = decoy;

			PLAYCTRL(true);

			play = true;

			while(play)
			{
				if(Bct<HALFSIZE)
				{
					f_read(&fil,&Buff[Bwi],HALFSIZE,&decoy);
					if((decoy!=HALFSIZE)||(!play_t))play=false;
					Bwi = (Bwi+decoy) & (BUFFERSIZE-1);
					__disable_irq();
					Bct+=decoy;
					__enable_irq();
				}
			}

			PLAYCTRL(false);

			Clear_Buffer();
			Bwi = 0;
			Bri  = 0;
		}
}

void TIMER16_0_IRQHandler (void)
{
		LPC_TMR32B1->MR0 = (*(int16_t*)(Buff+(Bri))+0x7FFF)>>7;
		LPC_TMR32B1->MR1 = (*(int16_t*)(Buff+(Bri+2))+0x7FFF)>>7;
		Bri = (Bri+4)&(BUFFERSIZE-1);
		Bct-=4;
		LPC_TMR32B1->TC=0;//clear timer counte

		Timer16_32_Clear_IR(&timer_conf);
}

void Clear_Buffer(void)
{
		uint32_t decoy;
		for(decoy=0;decoy<BUFFERSIZE;decoy++)
		Buff[decoy] = 0x10;
}

void SysTick_Handler(void)
{
		if(delay>0)delay--;

		if(play_t>0)play_t--;

		if(!PIR_STAT())	LED_GREEN = 0;
		else 					LED_GREEN = 1;

		disk_timerproc();	/* Disk timer process */
}

void Basic_Init(void)
{
		//****WARNING don't change this register LPC_IOCON->SWCLK_PIO0_10
		//****WARNING don't change this register LPC_IOCON->SWDIO_PIO1_3

		LPC_IOCON->PIO1_8 = 0x00;//statndard output and no pull up,down
		LPC_IOCON->PIO1_9 = 0x00;//statndard output and no pull up,down

		LPC_IOCON->PIO2_4 = 0x00;//statndard output and no pull up,down

		LPC_GPIO2->DIR &= ~(1<<4);//PIO2_4 is input mode for PIR
		LPC_GPIO1->DIR |= (1<<8)|(1<<9);//PIO1_8 and 1_9 is output mode for LEDs

		LPC_GPIO0->DATA = 0x00;//GPIO0 IS CLEARED
		LPC_GPIO1->DATA = 0x00;//GPIO1 IS CLEARED
		LPC_GPIO2->DATA = 0x00;//GPIO2 IS CLEARED
}


void ErrorHandle(void)
{
		LED_RED = 0;
		while(1);
}

void UART_Init_GPIO(void)
{
		LPC_IOCON->PIO1_6 &= ~(1<<0);
		LPC_IOCON->PIO1_6 |= (1<<0);
		LPC_IOCON->PIO1_7 &= ~(1<<0);
		LPC_IOCON->PIO1_7 |= (1<<0);
		LPC_GPIO1->DIR |= (1<<7);
		LPC_GPIO1->DIR &= ~(1<<6);
}

void SPI_Init_GPIO(void)
{
		LPC_IOCON->PIO0_8 = 0x01;//set PIO0_8 for MISO mode.
		LPC_IOCON->PIO0_9 = 0x01;//set PIO0_9 for MOSI mode.
		LPC_IOCON->SCK_LOC = 0x02;//set for SCK PIO0_6 pin .
		LPC_IOCON->PIO0_6= 0x02;//set PIO0_6 for SCK mode.
		LPC_GPIO0->DIR |= (1<<6);//set PIO0_6 for SCK mode.
		LPC_IOCON->PIO0_2 = 0x01;//set PIO0_2 for SSEL
}

