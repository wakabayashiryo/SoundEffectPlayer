/*
 * wav.h
 *
 *  Created on: 2016/02/27
 *      Author: evaota
 */

#ifndef WAV_WAV_H_
#define WAV_WAV_H_
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "../fatfs/diskio.h"
#include "../fatfs/ff.h"

/***
 *TODO:
 高速化　処理の整理
***/
#define BUFFSIZE 512
#define CHECKSIZE 512//the parameter must be 2 ^n (example:128 256 512)

#define SHOWDATA 0
#define DATANUM 500

typedef struct
{
  uint32_t FullSize;
  uint32_t DataSize;
}SizeWav_t;

typedef struct
{
  uint32_t id;
  int32_t size;
  int16_t format;
  uint16_t channels;
  uint32_t samplerate;
  uint32_t bytepersec;
  uint16_t blockalign;
  uint16_t bitswidth;
}FormatChunk_t;

typedef struct
{

}MUSICINFO;

int32_t CheckWavFile(FIL *fp,FormatChunk_t*header,SizeWav_t *sz);
int32_t DispWavInfo(FormatChunk_t *header,SizeWav_t *sz);
uint32_t GetMusicNum(DIR *dir,FILINFO *info);

#endif /* WAV_WAV_H_ */
