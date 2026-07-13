#include "stm32f10x.h"
#include "USART.h"
#include "key.h"

#define DATA_CAPTURE_KEY_PORT GPIOA
#define DATA_CAPTURE_KEY_PIN  GPIO_Pin_3
#define DATA_CAPTURE_KEY_DEBOUNCE_MS 30U
#define DATA_CAPTURE_KEY_LONG_PRESS_MS 2000U

typedef enum DataCaptureKeyEvent
{
	DATA_CAPTURE_KEY_EVENT_NONE = 0,
	DATA_CAPTURE_KEY_EVENT_SHORT,
	DATA_CAPTURE_KEY_EVENT_LONG
} DataCaptureKeyEvent_t;

static volatile DataCaptureKeyEvent_t DataCaptureKeyPending = DATA_CAPTURE_KEY_EVENT_NONE;
static volatile uint16_t DataCaptureKeyDurationMs = 0U;

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
	static uint8_t stablePressed = 0U;
	static uint8_t debounceCount = 0U;
	static uint16_t pressedDurationMs = 0U;
	uint8_t rawPressed;

	rawPressed = (GPIO_ReadInputDataBit(DATA_CAPTURE_KEY_PORT, DATA_CAPTURE_KEY_PIN) == Bit_RESET) ? 1U : 0U;
	if(rawPressed)
	{
		if(pressedDurationMs < 0xFFFFU)
		{
			pressedDurationMs++;
		}
		if(!stablePressed)
		{
			if(debounceCount < DATA_CAPTURE_KEY_DEBOUNCE_MS)
			{
				debounceCount++;
			}
			if(debounceCount >= DATA_CAPTURE_KEY_DEBOUNCE_MS)
			{
				stablePressed = 1U;
				debounceCount = 0U;
			}
		}
		else
		{
			debounceCount = 0U;
		}
		return;
	}

	if(!stablePressed)
	{
		debounceCount = 0U;
		pressedDurationMs = 0U;
		return;
	}

	if(debounceCount < DATA_CAPTURE_KEY_DEBOUNCE_MS)
	{
		debounceCount++;
	}
	if(debounceCount >= DATA_CAPTURE_KEY_DEBOUNCE_MS)
	{
		DataCaptureKeyDurationMs = pressedDurationMs;
		DataCaptureKeyPending = (pressedDurationMs >= DATA_CAPTURE_KEY_LONG_PRESS_MS) ?
			DATA_CAPTURE_KEY_EVENT_LONG : DATA_CAPTURE_KEY_EVENT_SHORT;
		stablePressed = 0U;
		debounceCount = 0U;
		pressedDurationMs = 0U;
	}
}

void DataCaptureKey_Service(void)
{
	DataCaptureKeyEvent_t pending;
	uint16_t durationMs;

	__disable_irq();
	pending = DataCaptureKeyPending;
	durationMs = DataCaptureKeyDurationMs;
	DataCaptureKeyPending = DATA_CAPTURE_KEY_EVENT_NONE;
	__enable_irq();

	if(pending == DATA_CAPTURE_KEY_EVENT_SHORT)
	{
		USART3_printf("CTR_PK SHORT DUR_MS=%u\r\n", durationMs);
	}
	else if(pending == DATA_CAPTURE_KEY_EVENT_LONG)
	{
		USART3_printf("CTR_REC LONG DUR_MS=%u\r\n", durationMs);
	}
}
