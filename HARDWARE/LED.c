#include "stm32f10x.h"

#include "LED.h"

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

	SetLED(GPIO_Pin_12,1);
	SetLED(GPIO_Pin_13,1);
	SetLED(GPIO_Pin_14,0);
}

//设置LED亮灭状态。
void SetLED(uint16_t GPIO_Pin,uint8_t V)
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

void SetLEDs(uint16_t GPIO_Pin)
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
