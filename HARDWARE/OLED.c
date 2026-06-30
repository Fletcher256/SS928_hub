#include "stm32f10x.h"
#include "OLED.h"
#include "OLED_Font.h"
#include "generic.h"

#define OLED_GPIO_PORT GPIOA
#define OLED_SCL_PIN   GPIO_Pin_4
#define OLED_SDA_PIN   GPIO_Pin_5

#ifndef OLED_WIDTH
#define OLED_WIDTH      128U
#endif

#ifndef OLED_HEIGHT
#define OLED_HEIGHT     64U
#endif

#ifndef OLED_PAGE_COUNT
#define OLED_PAGE_COUNT 8U
#endif

#ifndef OLED_IMAGE_SIZE
#define OLED_IMAGE_SIZE (OLED_WIDTH * OLED_PAGE_COUNT)
#endif

void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);

static uint8_t OLED_WriteAddress = 0x78;
static uint8_t OLED_DetectedAddress = 0;

//LVGL的image_converter与image_converter可以做字模转换

static void OLED_I2C_Delay(void)
{
	volatile uint8_t i;
	for(i = 0; i < 20; i++);
}

static void OLED_W_SCL(uint8_t Level)
{
	GPIO_WriteBit(OLED_GPIO_PORT, OLED_SCL_PIN, (BitAction)Level);
	OLED_I2C_Delay();
}

static void OLED_W_SDA(uint8_t Level)
{
	GPIO_WriteBit(OLED_GPIO_PORT, OLED_SDA_PIN, (BitAction)Level);
	OLED_I2C_Delay();
}

static uint8_t OLED_R_SDA(void)
{
	return GPIO_ReadInputDataBit(OLED_GPIO_PORT, OLED_SDA_PIN);
}

static void OLED_I2C_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = OLED_SCL_PIN | OLED_SDA_PIN;
	GPIO_Init(OLED_GPIO_PORT, &GPIO_InitStructure);

	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

static void OLED_I2C_Start(void)
{
	OLED_W_SDA(1);
	OLED_W_SCL(1);
	OLED_W_SDA(0);
	OLED_W_SCL(0);
}

static void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0);
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

static uint8_t OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	uint8_t AckBit;

	for(i = 0; i < 8; i++)
	{
		OLED_W_SDA((Byte & (0x80 >> i)) ? 1U : 0U);
		OLED_W_SCL(1);
		OLED_W_SCL(0);
	}

	OLED_W_SDA(1);
	OLED_W_SCL(1);
	AckBit = OLED_R_SDA();
	OLED_W_SCL(0);

	return (AckBit == 0) ? 1U : 0U;
}

static uint8_t OLED_CheckAddress(uint8_t WriteAddress)
{
	uint8_t Ack;

	OLED_I2C_Start();
	Ack = OLED_I2C_SendByte(WriteAddress);
	OLED_I2C_Stop();

	return Ack;
}

uint8_t OLED_Probe(void)
{
	OLED_I2C_Init();

	if(OLED_CheckAddress(0x78))
	{
		OLED_WriteAddress = 0x78;
		OLED_DetectedAddress = 0x3C;
		return OLED_DetectedAddress;
	}

	if(OLED_CheckAddress(0x7A))
	{
		OLED_WriteAddress = 0x7A;
		OLED_DetectedAddress = 0x3D;
		return OLED_DetectedAddress;
	}

	OLED_DetectedAddress = 0;
	return 0;
}

uint8_t OLED_GetAddress(void)
{
	return OLED_DetectedAddress;
}

static void OLED_WriteCommand(uint8_t Command)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(OLED_WriteAddress);
	OLED_I2C_SendByte(0x00);
	OLED_I2C_SendByte(Command);
	OLED_I2C_Stop();
}

static void OLED_WriteData(uint8_t Data)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(OLED_WriteAddress);
	OLED_I2C_SendByte(0x40);
	OLED_I2C_SendByte(Data);
	OLED_I2C_Stop();
}

static void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));
	OLED_WriteCommand(0x00 | (X & 0x0F));
}

void OLED_Clear(void)
{
	uint8_t i;
	uint8_t j;

	for(j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0);
		for(i = 0; i < 128; i++)
		{
			OLED_WriteData(0x00);
		}
	}
}

void OLED_ClearLine(uint8_t Line)
{
	OLED_ShowString(Line, 1, "                ");
}

void OLED_DrawBitmap128x64(const uint8_t *PageBitmap)
{
	uint8_t page;
	uint8_t x;

	if(PageBitmap == 0)
	{
		return;
	}

	for(page = 0; page < OLED_PAGE_COUNT; page++)
	{
		OLED_SetCursor(page, 0);
		for(x = 0; x < OLED_WIDTH; x++)
		{
			OLED_WriteData(PageBitmap[(uint16_t)page * OLED_WIDTH + x]);
		}
	}
}

void OLED_DrawMonoBitmap128x64(const uint8_t *RowMajorBitmap)
{
	uint8_t page;
	uint8_t x;
	uint8_t bit;
	uint8_t data;
	uint16_t rowIndex;

	if(RowMajorBitmap == 0)
	{
		return;
	}

	for(page = 0; page < OLED_PAGE_COUNT; page++)
	{
		OLED_SetCursor(page, 0);
		for(x = 0; x < OLED_WIDTH; x++)
		{
			data = 0;
			for(bit = 0; bit < 8; bit++)
			{
				rowIndex = (uint16_t)(page * 8U + bit) * (OLED_WIDTH / 8U) + (x / 8U);
				if((RowMajorBitmap[rowIndex] & (uint8_t)(0x80U >> (x & 0x07U))) != 0)
				{
					data |= (uint8_t)(1U << bit);
				}
			}
			OLED_WriteData(data);
		}
	}
}

void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
	uint8_t i;
	uint8_t Index;

	if(Char < ' ' || Char > '~')
	{
		Char = ' ';
	}

	Index = (uint8_t)(Char - ' ');
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
	for(i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Index][i]);
	}

	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
	for(i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Index][i + 8]);
	}
}

void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;

	for(i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}
}

static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;

	while(Y--)
	{
		Result *= X;
	}

	return Result;
}

void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;

	for(i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i, (char)(Number / OLED_Pow(10, Length - i - 1) % 10 + '0'));
	}
}

void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;

	if(Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');
		Number1 = (uint32_t)Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');
		Number1 = (uint32_t)(-Number);
	}

	for(i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i + 1, (char)(Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0'));
	}
}

void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	uint8_t SingleNumber;

	for(i = 0; i < Length; i++)
	{
		SingleNumber = (uint8_t)(Number / OLED_Pow(16, Length - i - 1) % 16);
		if(SingleNumber < 10)
		{
			OLED_ShowChar(Line, Column + i, (char)(SingleNumber + '0'));
		}
		else
		{
			OLED_ShowChar(Line, Column + i, (char)(SingleNumber - 10 + 'A'));
		}
	}
}

void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;

	for(i = 0; i < Length; i++)
	{
		OLED_ShowChar(Line, Column + i, (char)(Number / OLED_Pow(2, Length - i - 1) % 2 + '0'));
	}
}

void OLED_Init(void)
{
	Delay_ms(100);

	if(OLED_Probe() == 0)
	{
		OLED_WriteAddress = 0x78;
	}

	OLED_WriteCommand(0xAE);
	OLED_WriteCommand(0xD5);
	OLED_WriteCommand(0x80);
	OLED_WriteCommand(0xA8);
	OLED_WriteCommand(0x3F);
	OLED_WriteCommand(0xD3);
	OLED_WriteCommand(0x00);
	OLED_WriteCommand(0x40);
	OLED_WriteCommand(0xA1);
	OLED_WriteCommand(0xC8);
	OLED_WriteCommand(0xDA);
	OLED_WriteCommand(0x12);
	OLED_WriteCommand(0x81);
	OLED_WriteCommand(0xCF);
	OLED_WriteCommand(0xD9);
	OLED_WriteCommand(0xF1);
	OLED_WriteCommand(0xDB);
	OLED_WriteCommand(0x30);
	OLED_WriteCommand(0xA4);
	OLED_WriteCommand(0xA6);
	OLED_WriteCommand(0x8D);
	OLED_WriteCommand(0x14);
	OLED_WriteCommand(0xAF);

	OLED_Clear();
}
