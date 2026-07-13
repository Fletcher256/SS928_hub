#include "stm32f10x.h"

#include "LED.h"

LED_STATE CurrentLedState = LED_STATE_GREEN;

#define LED_BLINK_HALF_PERIOD_MS 400U

static uint8_t LedBlinkEnabled = 0U;
static uint8_t LedBlinkOnPhase = 1U;
static uint16_t LedBlinkElapsedMs = 0U;

void LED_Init()
{
	//APB2外设挂载的GPIO1口的RCC时钟使能端:开启。之后这个IO口就使用这个时钟信号来对IO口进行控制。
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	//额这里用函数配置接口的话难道不是用一次就没了(
	GPIO_InitTypeDef InitGPIOB ;
	
	InitGPIOB.GPIO_Mode = GPIO_Mode_Out_PP;
	
	//设置GPIO口的引脚编号
	//梭哈全开A口PIN引脚
	//这个PIN口如果定义过之后,后面重定义其他口不会将前面的设置覆盖掉。
	
	InitGPIOB.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	
	//设置这个GPIO口的时钟频率?
	InitGPIOB.GPIO_Speed = GPIO_Speed_50MHz;
	
	//初始化整个GPIO口。
	GPIO_Init(GPIOB,&InitGPIOB);

	LED_SetState(LED_STATE_GREEN);
}

//设置LED亮灭状态。
static void SetLED(uint16_t GPIO_Pin,uint8_t V)
{
	if(V)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin);
	}
	else
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin);
	}
}

static void SetLEDs(uint16_t GPIO_Pin)
{
	SetLED(GPIO_Pin,0);
	switch(GPIO_Pin)
	{
		case GPIO_Pin_12:
		{
			SetLED(GPIO_Pin_13,1);
			SetLED(GPIO_Pin_14,1);
			break;
		}
		case GPIO_Pin_13:
		{
			SetLED(GPIO_Pin_12,1);
			SetLED(GPIO_Pin_14,1);
			break;
		}
		case GPIO_Pin_14:
		{
			SetLED(GPIO_Pin_12,1);
			SetLED(GPIO_Pin_13,1);
			break;
		}
	}
	
}

static void SetAllLEDsOff(void)
{
	/* LEDs are active-low on PB12/PB13/PB14. */
	SetLED(GPIO_Pin_12, 1);
	SetLED(GPIO_Pin_13, 1);
	SetLED(GPIO_Pin_14, 1);
}

static void ApplyCurrentLedOutput(void)
{
	if(LedBlinkEnabled && !LedBlinkOnPhase)
	{
		SetAllLEDsOff();
		return;
	}

	switch(CurrentLedState)
	{
		case LED_STATE_YELLOW:
			SetLEDs(GPIO_Pin_12);
			break;
		case LED_STATE_RED:
			SetLEDs(GPIO_Pin_13);
			break;
		case LED_STATE_GREEN:
		default:
			SetLEDs(GPIO_Pin_14);
			break;
	}
}

void LED_SetState(LED_STATE state)
{
	CurrentLedState = state;
	ApplyCurrentLedOutput();
}

void LED_SetBlink(uint8_t enabled)
{
	LedBlinkEnabled = enabled ? 1U : 0U;
	LedBlinkOnPhase = 1U;
	LedBlinkElapsedMs = 0U;
	ApplyCurrentLedOutput();
}

uint8_t LED_IsBlinking(void)
{
	return LedBlinkEnabled;
}

void LED_BlinkService1ms(void)
{
	if(!LedBlinkEnabled)
	{
		return;
	}
	if(LedBlinkElapsedMs < LED_BLINK_HALF_PERIOD_MS)
	{
		LedBlinkElapsedMs++;
	}
	if(LedBlinkElapsedMs >= LED_BLINK_HALF_PERIOD_MS)
	{
		LedBlinkElapsedMs = 0U;
		LedBlinkOnPhase = LedBlinkOnPhase ? 0U : 1U;
		ApplyCurrentLedOutput();
	}
}
