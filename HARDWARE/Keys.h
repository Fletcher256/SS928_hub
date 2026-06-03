#ifndef _KEY_H_
#define _KEY_H_

typedef unsigned char UCHAR;

typedef unsigned short USHORT;

#include "stdarg.h"

typedef enum 
{
	PB0 = 0,
	PB1 ,
	PB2 ,
	PB3 ,
	PB4 ,
	PB5 ,
	PB6 ,
	PB7 ,
	PB8 ,
	PB9 ,
	PB10 ,
	PB11 ,
	PB12 ,
	PB13 ,
	PB14 ,
	PB15
}KEY_PORT;
//原始的延时,消抖。。。

uint8_t UOB_CheckKeyState(void);

uint8_t GetPressedKey(void);

void UOB_KeyFrame(void);

void Key_Init(UCHAR KEYNum,...);

int8_t CheckKey(UCHAR KEYNum);

int8_t CheckKeys(void);

#endif
