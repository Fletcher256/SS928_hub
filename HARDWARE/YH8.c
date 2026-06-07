#include "stm32f10x.h"

#include "YH8.h"

void YHB_Init()
{
	//APB2外设挂载的GPIO1口的RCC时钟使能端:开启。之后这个IO口就使用这个时钟信号来对IO口进行控制。
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	//额这里用函数配置接口的话难道不是用一次就没了(
	GPIO_InitTypeDef InitGPIOA ;
	
	InitGPIOA.GPIO_Mode = GPIO_Mode_Out_OD;
	
	//设置GPIO口的引脚编号
	//梭哈全开A口PIN引脚
	//这个PIN口如果定义过之后,后面重定义其他口不会将前面的设置覆盖掉。
	
	InitGPIOA.GPIO_Pin = GPIO_Pin_2;
	
	//设置这个GPIO口的时钟频率?
	InitGPIOA.GPIO_Speed = GPIO_Speed_50MHz;
	
	//初始化整个GPIO口。
	GPIO_Init(GPIOA,&InitGPIOA);

	SetYH8(0);
}

//设置RS0102YH8转换状态。
void SetYH8(uint8_t V)
{
	if(V)
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_2);
	}
	else
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	}
}
