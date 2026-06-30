#ifndef __OLED_H
#define __OLED_H

#include "stm32f10x.h"

#define OLED_WIDTH      128U
#define OLED_HEIGHT     64U
#define OLED_PAGE_COUNT 8U
#define OLED_IMAGE_SIZE (OLED_WIDTH * OLED_PAGE_COUNT)

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ClearLine(uint8_t Line);
uint8_t OLED_Probe(void);
uint8_t OLED_GetAddress(void);
void OLED_DrawBitmap128x64(const uint8_t *PageBitmap);
void OLED_DrawMonoBitmap128x64(const uint8_t *RowMajorBitmap);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#endif
