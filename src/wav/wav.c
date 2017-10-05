/*
 * wav.c
 *
 *  Created on: 2016/02/27
 *      Author: evaota
 */
#include "wav.h"
#include "../xprintf.h"

static uint16_t CastUint16(char *Cval)
{
  return (Cval[1]<<8)+Cval[0];
}

static uint32_t CastUint32(char *Cval)
{
  return ((Cval[3]<<24)+(Cval[2]<<16)+(Cval[1]<<8)+Cval[0]);
}

int32_t CheckWavFile(FIL*fp,FormatChunk_t *header,SizeWav_t*sz)
{
  uint32_t count = 0,datachpos = 0,rn;
  char wb[BUFFSIZE];//work buff
  char *point;
  uint32_t sec,min,recheck=0;

#if SHOWDATA
  uint8_t c;
  for(int i=0;i<DATANUM;i++)
    {
      c = fgetc(fp);
      if(('a'<=c&&c<='z')||('A'<=c&&c<='Z'))
	xprintf(" %c",c);
      else
	xprintf(" 0x%02x",c);
      if((i%8)==7)xprintf("\n");
    }
#else
	if(f_read(fp,wb,CHECKSIZE,&rn)!=FR_OK)return-1;
	if(rn!=CHECKSIZE)return -1;

  if(strstr(wb,"RIFF")!=wb)return -2;
  if(strstr(wb,"WAVE")!=(wb+8))return -2;
  sz->FullSize = CastUint32(wb+4)+8;

  do{
    if(recheck)
    {
    	if(f_read(fp,wb,CHECKSIZE,&rn)!=FR_OK)return-1;
    	if(rn!=CHECKSIZE)return -1;
    }
    recheck = 1;
    for(count=12;count<CHECKSIZE;count++)
      {
	if(( ('a'<=wb[count]) && (wb[count]<='z') ) || ( ('A'<=wb[count]) && (wb[count]<='Z') ))
	  {
	    if((point=strstr(wb+count,"fmt "))!=NULL)
	      {
		//    xprintf("get %s chunk\n",point);
		count += CastUint32(point+4);
		point += 8;
		header->format = CastUint16(point);
		header->channels = CastUint16(point+2);
		header->samplerate = CastUint32(point+4);
		header->bytepersec = CastUint32(point+8);
		header->blockalign = CastUint16(point+12);
		header->bitswidth = CastUint16(point+14);
	      }
	    else if((point=strstr(wb+count,"fact"))!=NULL)
	      {
		//  xprintf("get %s chunk\n",point);
		count += CastUint32(point+4);
	      }
	    else if((point=strstr(wb+count,"data"))!=NULL)
	      {
		// xprintf("get %s chunk\n",point);
		recheck = 0;
		sz->DataSize = CastUint32(point+4);
		datachpos += count+10;
		break;
	      }

	    else if((point=strstr(wb+count,"LIST"))!=NULL)
	      {
		//   xprintf("get %s chunk\n",point);
	      }
	    else if((point=strstr(wb+count,"INFO"))!=NULL)
	      {
		//     xprintf("get %s chunk\n",point);
	      }
	    else if((point=strstr(wb+count,"ITRK"))!=NULL)
	      {
		//    xprintf("get %s chunk\n",point);
		count += CastUint32(point+4);
		xprintf("Track Number is %d\n",point[8]+(point[9]<<8));
	      }
	    else if((point=strstr(wb+count,"IGNR"))!=NULL)
	      {
		///   xprintf("get %s chunk\n",point);
		count += CastUint32(point+4);
		xprintf("Genre is %s\n",point+8);
	      }
	    else if((point=strstr(wb+count,"IART"))!=NULL)
	      {
		//   xprintf("get %s chunk\n",point);
		count += CastUint32(point+4);
		xprintf("Artist is %s\n",point+8);
	      }
	    else if((point=strstr(wb+count,"ICRD"))!=NULL)
	      {
		// xprintf("get %s chunk\n",point);
		count += CastUint32(point+4);
	      }
	    else if((point=strstr(wb+count,"IFNR"))!=NULL)
	      {
		//   xprintf("get %s chunk\n",point);
		count += CastUint32(point+4);
	      }
	    else if((point=strstr(wb+count,"INAM"))!=NULL)
	      {
		//  xprintf("get %s chunk\n",point);
		count += CastUint32(point+4);
		xprintf("Song Name is %s\n",point+8);
	      }
	    else if((point=strstr(wb+count,"IPRD"))!=NULL)
	      {
		//  xprintf("get %s chunk\n",point);
		count += CastUint32(point+4);
		xprintf("Album Title is %s\n",point+8);
	      }
	  }
      }
    if(recheck)datachpos += count;
  }while(recheck);
  min = sz->DataSize / header->bytepersec / 60;
  sec = (sz->DataSize / header->bytepersec) - (min * 60);
  xprintf("time:%02d:%02d\n",min,sec);
#endif
  return 1;
}

int32_t DispWavInfo(FormatChunk_t*header,SizeWav_t *sz)
{
  if((header==NULL)||(sz==NULL))return -1;
  xprintf("/*****HeaderInformation*****/");
  xprintf("\n format = %d\n channels = %d\n samplerate = %d\n bytepersec = %d\n blockalign = %d\n bitswidth = %d\n",header->format,header->channels,header->samplerate,header->bytepersec,header->blockalign,header->bitswidth);
  xprintf("/*****WavSizeInformation*****/");
  xprintf("\n wav file size  = %dByte\n",sz->FullSize);
  xprintf(" data chunksize = %dByte\n",sz->DataSize);

  return 0;
}

uint32_t GetMusicNum(DIR *dir,FILINFO *info)
{
	uint32_t num = 0;
	FRESULT res;

	res = f_opendir(dir,"");
	if(res!=FR_OK)
	{
		xprintf("FatFs opendir error\n");
		return 0;
	}
	do{
		res = f_readdir(dir,info);
		if(!((res!=FR_OK)||(strstr(info->fname,".WAV")==NULL)))num++;
	}while(info->fname[0]);
	f_closedir(dir);
	xprintf("music number is %d in SD card.\n",num);
	return num;
}

