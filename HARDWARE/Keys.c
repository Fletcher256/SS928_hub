#include "stm32f10x.h"

#include "Keys.h"

#include "generic.h"

GPIO_InitTypeDef InitGPIOB;

uint8_t lastState = 16;

uint8_t preState = 16;

uint8_t KeyNum = 16;

void Key_Init(UCHAR KEYNum,...)
{
	va_list args;
	
   va_start(args, KEYNum);

	//APB2外设挂载的GPIO1口的RCC时钟使能端:开启。之后这个IO口就使用这个时钟信号来对IO口进行控制。
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	//额这里用函数配置接口的话难道不是用一次就没了(
	
	//开关按下接高电平,因此这里我们配置输入口为下拉
	InitGPIOB.GPIO_Mode = GPIO_Mode_IPD;
	
	//设置GPIO口的引脚编号
	//梭哈全开A口PIN引脚
	//这个PIN口如果定义过之后,后面重定义其他口不会将前面的设置覆盖掉。
	
	InitGPIOB.GPIO_Pin = 0X00000000;
	
	for(int i = 0;i<KEYNum;i++)
	{
			InitGPIOB.GPIO_Pin |= GPIO_Pin_0 << va_arg(args, int);
	}

	va_end(args);
	
	//设置这个GPIO口的时钟频率
	//作为读输入的IO口,其实吧这个频率也不是很重要。。。
	InitGPIOB.GPIO_Speed = GPIO_Speed_50MHz;
	
	//初始化整个GPIO口。
	GPIO_Init(GPIOB,&InitGPIOB);
}

//单个按键检测
//有时候更加符合应用场景。
//这个根据需要来修改里面的枚举即可。
int8_t CheckKey(UCHAR KEYNum)
{
	int16_t t = GPIO_Pin_0 << KEYNum;
	if((InitGPIOB.GPIO_Pin & t)  && GPIO_ReadInputDataBit(GPIOB,t) == 1)
		{
			Delay_ms(20);while(GPIO_ReadInputDataBit(GPIOB,t) == 1){}Delay_ms(20);return KEYNum;
		}
	return 16;
}

//用定时器实现非阻塞式按键弹起触发功能
uint8_t UOB_CheckKeyState()
{
	//获得当前是否按键按下。
	
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0 <<PB9) == 1)
	{
		return 9;
	}
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0 <<PB15) == 1)
	{
		return 15;
	}
	
	return 16;
}

uint8_t GetPressedKey()
{
	uint8_t t;
	//这一步是为了防止按键进入本函数时中断来了把KeyNum改成16,导致这一次本函数执行失效(算一个缓冲区提高容错用的)
	if(KeyNum != 16)
	{
		t = KeyNum;
		KeyNum = 16;
		return t;
	}

	return 16;
}

//按键帧,在中断里检测是否有按键下降沿读入。
void UOB_KeyFrame()
{
	//获得当前按键情况
	lastState = preState;
	
	preState = UOB_CheckKeyState();
	
	//检测弹起(上升沿,即上一次状态为按下,当前状态为弹起,则算一个上升沿。)
	if(preState == 16 && lastState != 16)
	{
		 KeyNum = lastState;
	}
}

//这里需要打表。比较麻烦。
int8_t CheckKeys()
{
	//好像这里不能枚举太多GPIO_ReadInputDataBit似乎这个函数执行要花挺多时间。。
	//不是如此的。因为那几个引脚都没配置。。所以用这个检测函数是会出错的
	//只有配置过的引脚才能正常读取吧。。
	
	//没有配置过的口是无法检测的。所以这里完全可以用枚举来检测。
	//没配置的口会出现未定义行为(
	if((InitGPIOB.GPIO_Pin & GPIO_Pin_0)  && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) == 1){Delay_ms(20);while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) == 1){}Delay_ms(20);return 0;}
	if((InitGPIOB.GPIO_Pin & GPIO_Pin_1)  && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) == 1){Delay_ms(20);while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) == 1){}Delay_ms(20);return 1;}
	if((InitGPIOB.GPIO_Pin & GPIO_Pin_2)  && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2) == 1){Delay_ms(20);while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2) == 1){}Delay_ms(20);return 2;}
	if((InitGPIOB.GPIO_Pin & GPIO_Pin_3)  && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3) == 1){Delay_ms(20);while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3) == 1){}Delay_ms(20);return 3;}
	
	if((InitGPIOB.GPIO_Pin & GPIO_Pin_4)  && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4) == 1){Delay_ms(20);while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4) == 1){}Delay_ms(20);return 4;}
	if((InitGPIOB.GPIO_Pin & GPIO_Pin_5)  && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5) == 1){Delay_ms(20);while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5) == 1){}Delay_ms(20);return 5;}
	if((InitGPIOB.GPIO_Pin & GPIO_Pin_6)  && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6) == 1){Delay_ms(20);while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6) == 1){}Delay_ms(20);return 6;}
	if((InitGPIOB.GPIO_Pin & GPIO_Pin_7)  && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7) == 1){Delay_ms(20);while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7) == 1){}Delay_ms(20);return 7;}
	
	if((InitGPIOB.GPIO_Pin & GPIO_Pin_8)  && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8) == 1){Delay_ms(20);while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8) == 1){}Delay_ms(20);return 8;}
	if((InitGPIOB.GPIO_Pin & GPIO_Pin_9)  && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9) == 1){Delay_ms(20);while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9) == 1){}Delay_ms(20);return 9;}
	if((InitGPIOB.GPIO_Pin & GPIO_Pin_10)  && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10) == 1){Delay_ms(20);while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10) == 1){}Delay_ms(20);return 10;}
	if((InitGPIOB.GPIO_Pin & GPIO_Pin_11)  && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11) == 1){Delay_ms(20);while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11) == 1){}Delay_ms(20);return 11;}
	
	if((InitGPIOB.GPIO_Pin & GPIO_Pin_12)  && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12) == 1){Delay_ms(20);while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12) == 1){}Delay_ms(20);return 12;}
	if((InitGPIOB.GPIO_Pin & GPIO_Pin_13)  && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13) == 1){Delay_ms(20);while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_13) == 1){}Delay_ms(20);return 13;}
	if((InitGPIOB.GPIO_Pin & GPIO_Pin_14)  && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14) == 1){Delay_ms(20);while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14) == 1){}Delay_ms(20);return 14;}
	if((InitGPIOB.GPIO_Pin & GPIO_Pin_15)  && GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15) == 1){Delay_ms(20);while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15) == 1){}Delay_ms(20);return 15;}
	return 16;
}
