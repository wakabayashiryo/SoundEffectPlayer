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
#include "xprintf.h"
#include "main.h"
#include "wav/wav.h"
#include "fatfs/diskio.h"
#include "fatfs/ff.h"
#include <cr_section_macros.h>

#define BUFFERSIZE (512*2)
#define HALFSIZE (BUFFERSIZE>>1)
uint8_t Buff[BUFFERSIZE];

#define PLAYCTRL(x) ((x==true)? NVIC_EnableIRQ(TIMER_16_0_IRQn):NVIC_DisableIRQ(TIMER_16_0_IRQn))

uint32_t Bri=0,Bwi=0,Bct=0;

extern void disk_timerproc (void);

void Basic_Init(void);
void Clear_Buffer(void);
void ErrorHandle(uint8_t blinknum,const char *message);

void UART_Init_GPIO(void);

volatile uint32_t Timer;

LPC_TMRx_Config timer_conf;
LPC_PWM_Config pconfig;

volatile uint32_t delay;
#define LPC_delay(time) {\
		delay = time;\
		while(delay);\
}

volatile bool wav_ok=false,play = false,playctrl = true;

int main(void)
{
		DIR dir;
		FILINFO info;

		FATFS fs;
		FIL fil;

		FormatChunk_t FmCk;
		SizeWav_t WvSz;
		uint32_t decoy,musicnum,nowtrack = 0,skipcount = 0;

		Basic_Init();

		LED_BLUE = 1;
		LED_RED= 1;
		LED_GREEN = 1;

		UART_Init(UART_Init_GPIO,BAUD_9600);
		xdev_out(UART_Transmit);
		xdev_in(UART_Receive);

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
				ErrorHandle(1,"FatFs mount error\n");

		if(f_mount(&fs,"",0)!=FR_OK)
				ErrorHandle(2,"FatFs initialize error\n");

		musicnum = GetMusicNum(&dir,&info);
		if(musicnum<0)
				ErrorHandle(3,"Read wav header error\n");

		while(1)
		{
			   if(f_opendir(&dir,"")==FR_OK)wav_ok=true;
				else wav_ok = false;

				while(wav_ok)
				{
					do
					{
							if(skipcount>0)skipcount--;
							if(f_readdir(&dir,&info)!=FR_OK)skipcount++;
							if(strstr(info.fname,".WAV")==NULL)
							  skipcount++;
					}while(skipcount);

					if(f_open(&fil,info.fname,FA_READ)!=FR_OK)break;

					CheckWavFile(&fil,&FmCk,&WvSz);
					DispWavInfo(&FmCk,&WvSz);

					if(f_read(&fil,&Buff[0],BUFFERSIZE,&decoy)!=FR_OK)break;
					Bct = decoy;

					nowtrack++;
					play = true;

					PLAYCTRL(true);

					while(play)
					{
					  if(SW_HOLD())
					  {
						  if(!SW_RIGHT())
						  {
							  play = false;
							  if(nowtrack==musicnum)
							  {
								  f_closedir(&dir);
								  wav_ok = false;
								  nowtrack = 0;
							  }
						  }
						  else if(!SW_LEFT())
						  {
							  if(nowtrack==1)
							  {
								  nowtrack = musicnum-1;
								  skipcount = musicnum;
							  }
							  else
							  {
								  nowtrack -=2;
								  skipcount = nowtrack+1;
							  }
							  f_closedir(&dir);
							  play = false;
							  wav_ok = false;
						  }

						  if(!SW_CENTER()&&(Timer>250))
						  {
							  Timer = 0;
							  if(playctrl)
							  {
								  playctrl =false;
								  PLAYCTRL(false);
							  }
							  else
							  {
								  playctrl = true;
								  PLAYCTRL(true);
							  }
						  }
					  }

					  if(Bct<HALFSIZE)
					  {
						  f_read(&fil,&Buff[Bwi],HALFSIZE,&decoy);
						  if(decoy!=HALFSIZE)play=false;
						  Bwi = (Bwi+decoy) & (BUFFERSIZE-1);
						  __disable_irq();
						  Bct+=decoy;
						  __enable_irq();
					  }
					}

					if(nowtrack==musicnum)
					{
					  wav_ok = false;
					  nowtrack = 1;
					}
					PLAYCTRL(false);

					Clear_Buffer();
					Bwi = 0;
					Bri  = 0;
				}
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
		static uint32_t led = 0;

		led++;

		if((led>1000)&&playctrl)
		{
				led = 0;
				LED_RED = 1;		//Celar RED of LED
				LED_BLUE ^= 1;	//toggle Bule of LED ever 1second
		}
		else if(!playctrl)
		{
			    LED_BLUE = 1;		//Clear Bule of LED
				LED_RED = 0;		//Set red of LED
		}

		if(delay>0)delay--;

		if((Timer<60000)&&SW_CENTER())Timer++;
		disk_timerproc();	/* Disk timer process */
}

void Basic_Init(void)
{
		LPC_IOCON->PIO0_1 = 0x10;//control playing music(Play)[pull up mode]
		LPC_IOCON->PIO0_3 = 0x10;//control lock[pull up mode]
		LPC_IOCON->PIO0_4 = 0x00;
		LPC_IOCON->PIO0_5 = 0x00;
		//****WARNING don't change this register LPC_IOCON->SWCLK_PIO0_10
		//****WARNING don't change this register LPC_IOCON->SWDIO_PIO1_3
		LPC_IOCON->PIO1_4 = 0x00;//indicator LED
		LPC_IOCON->PIO1_8 = 0x10;//control playing music(stop/start)[pull up mode]
		LPC_IOCON->PIO1_9 = 0x10;//control playing music(Back)[pull up mode]

		LPC_GPIO0->DIR = (1<<4)|(1<<5);//
		LPC_GPIO1->DIR = (1<<4);

		LPC_GPIO0->DATA = 0x00;//GPIO0 IS CLEARED
		LPC_GPIO1->DATA = 0x00;//GPIO1 IS CLEARED
}


void ErrorHandle(uint8_t blinknum,const char *message)
{
		xprintf(message);
		for(uint32_t i = 0; i<blinknum;i++)
		{
			LED_RED = 1;
			LPC_delay(1000);
			LED_RED = 0;
			LPC_delay(1000);
		}
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

