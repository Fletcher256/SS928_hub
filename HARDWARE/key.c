#include "stm32f10x.h"
#include "CarControl.h"
#include "key.h"

#define DATA_CAPTURE_KEY_PORT GPIOA
#define DATA_CAPTURE_KEY_PIN  GPIO_Pin_3
#define DATA_CAPTURE_KEY_DEBOUNCE_MS 30U

static volatile uint8_t DataCaptureKeyPending = 0;

void DataCaptureKey_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = DATA_CAPTURE_KEY_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DATA_CAPTURE_KEY_PORT, &GPIO_InitStructure);
}

void DataCaptureKey_SysTick(void)
{
	static uint8_t debounceActive = 0;
	static uint8_t pressedLatched = 0;
	static uint8_t debounceCount = 0;
	uint8_t rawPressed;

	rawPressed = (GPIO_ReadInputDataBit(DATA_CAPTURE_KEY_PORT, DATA_CAPTURE_KEY_PIN) == Bit_RESET) ? 1U : 0U;
	if(!rawPressed)
	{
		debounceActive = 0;
		debounceCount = 0;
		pressedLatched = 0;
		return;
	}

	if(pressedLatched)
	{
		return;
	}

	if(!debounceActive)
	{
		debounceActive = 1;
		debounceCount = 0;
		return;
	}

	if(debounceCount < DATA_CAPTURE_KEY_DEBOUNCE_MS)
	{
		debounceCount++;
	}
	if(debounceCount >= DATA_CAPTURE_KEY_DEBOUNCE_MS)
	{
		pressedLatched = 1;
		debounceActive = 0;
		DataCaptureKeyPending = 1;
	}
}

void DataCaptureKey_Service(void)
{
	uint8_t pending;

	__disable_irq();
	pending = DataCaptureKeyPending;
	DataCaptureKeyPending = 0;
	__enable_irq();

	if(pending)
	{
		PrintTelemetry();
	}
}
