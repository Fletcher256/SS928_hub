#include "stm32f10x.h"
#include "PMW3901_SoftSPI.h"
#include "generic.h"

#define PMW3901_SCK_PORT      GPIOB
#define PMW3901_SCK_PIN       GPIO_Pin_8
#define PMW3901_MOSI_PORT     GPIOB
#define PMW3901_MOSI_PIN      GPIO_Pin_9
#define PMW3901_MISO_PORT     GPIOB
#define PMW3901_MISO_PIN      GPIO_Pin_15
#define PMW3901_CS_PORT       GPIOA
#define PMW3901_CS_PIN        GPIO_Pin_0

#define PMW3901_SCK_HIGH()    (PMW3901_SCK_PORT->BSRR = PMW3901_SCK_PIN)
#define PMW3901_SCK_LOW()     (PMW3901_SCK_PORT->BRR = PMW3901_SCK_PIN)
#define PMW3901_MOSI_HIGH()   (PMW3901_MOSI_PORT->BSRR = PMW3901_MOSI_PIN)
#define PMW3901_MOSI_LOW()    (PMW3901_MOSI_PORT->BRR = PMW3901_MOSI_PIN)
#define PMW3901_CS_HIGH()     (PMW3901_CS_PORT->BSRR = PMW3901_CS_PIN)
#define PMW3901_CS_LOW()      (PMW3901_CS_PORT->BRR = PMW3901_CS_PIN)

static void PMW3901_SoftSPI_DelayHalfCycle(void)
{
	/* 1 us half-cycle keeps SCK well below the PMW3901 2 MHz limit. */
	Delay_us(1U);
}

void PMW3901_SoftSPI_Init(void)
{
	GPIO_InitTypeDef gpio;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

	PMW3901_CS_HIGH();
	PMW3901_SCK_HIGH();
	PMW3901_MOSI_LOW();

	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;

	gpio.GPIO_Pin = PMW3901_CS_PIN;
	GPIO_Init(PMW3901_CS_PORT, &gpio);

	gpio.GPIO_Pin = PMW3901_SCK_PIN | PMW3901_MOSI_PIN;
	GPIO_Init(PMW3901_SCK_PORT, &gpio);

	/* Pull-up makes an open MISO line visible as 0xFF instead of 0x00. */
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	gpio.GPIO_Pin = PMW3901_MISO_PIN;
	GPIO_Init(PMW3901_MISO_PORT, &gpio);

	PMW3901_SoftSPI_Deselect();
}

void PMW3901_SoftSPI_Select(void)
{
	PMW3901_SCK_HIGH();
	PMW3901_CS_LOW();
	PMW3901_SoftSPI_DelayHalfCycle();
}

void PMW3901_SoftSPI_Deselect(void)
{
	PMW3901_SCK_HIGH();
	PMW3901_CS_HIGH();
	PMW3901_SoftSPI_DelayHalfCycle();
}

uint8_t PMW3901_SoftSPI_Transfer(uint8_t txData)
{
	uint8_t rxData = 0;
	uint8_t mask;

	for(mask = 0x80U; mask != 0U; mask >>= 1)
	{
		PMW3901_SCK_LOW();
		if((txData & mask) != 0U)
		{
			PMW3901_MOSI_HIGH();
		}
		else
		{
			PMW3901_MOSI_LOW();
		}
		PMW3901_SoftSPI_DelayHalfCycle();

		PMW3901_SCK_HIGH();
		PMW3901_SoftSPI_DelayHalfCycle();
		if(GPIO_ReadInputDataBit(PMW3901_MISO_PORT, PMW3901_MISO_PIN) != Bit_RESET)
		{
			rxData |= mask;
		}
	}

	return rxData;
}

uint8_t PMW3901_SoftSPI_ReadMisoLevel(void)
{
	return (GPIO_ReadInputDataBit(PMW3901_MISO_PORT, PMW3901_MISO_PIN) != Bit_RESET) ? 1U : 0U;
}
