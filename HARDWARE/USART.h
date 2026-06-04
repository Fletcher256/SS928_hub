#ifndef _USART_H_
#define _USART_H_

#define DPACKAGE_HEAD 0XFF

#define DPACKAGE_TAIL 0XFE

#define TPACKAGE_HEAD '@'

#define TPACKAGE_TAIL0 '\r'

#define TPACKAGE_TAIL1 '\n'

#include <stdio.h>

#include <stdarg.h>

extern const char * COMMANDS[];

void USART3_Init(void);

void USART3_SendByte(uint8_t Byte);

void USART3_SendArray(int8_t Array[],uint32_t LEN);

void USART3_SendString(const char Array[]);

void USART3_SendNum(int32_t num);

void USART3_SendSingedNum(int32_t num);

uint8_t USART_ReadByte(void);

void USART3_printf(const char * format,...);

uint16_t GetUSART3Data(void);

//uint8_t GetUSART3RXFlag(void);

void SendArrayPackage(int8_t Array[],uint32_t LEN);

void SendStringPackage(const char Array[]);

int8_t * GetUSART3DataBuffer(void);

uint16_t GetUSART3BufferCnt(void);

uint8_t GetUSART3RXDState(void);

char * GetUSART3TextBuffer(void);

uint8_t GetUSART3RXTState(void);

#endif
