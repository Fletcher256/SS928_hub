#include "stm32f10x.h"

#include "Delay.h"

int Ec_div = 0;

uint8_t is_clockwise = 0;

int EcSpeed = 0;

//向右自增,向左自减
void RotaryEncoder_Init()
{
	GPIO_InitTypeDef InitGPIOB;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	InitGPIOB.GPIO_Mode = GPIO_Mode_IPU;
	
	InitGPIOB.GPIO_Pin = 0X00000000;

	//我们这里使用的是TIM4的CH1与CH2
	InitGPIOB.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	
	InitGPIOB.GPIO_Speed = GPIO_Speed_50MHz;
	
	//初始化整个GPIO口。
	//修正这个说法。这里其实是初始化你选中这个GPIO组的这几个PIN口,不是全部扫描。
	GPIO_Init(GPIOA,&InitGPIOB);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
		
	//外部时钟托管TIM4的CNT行为无需内部时钟干预。
	
	//------------以下配置定时器的计时模块
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	//这个时钟分频是外部滤波器的采样频率,它可以直接由内部的时钟控制,频率越小,采样越慢,滤波效果越好。若发现杂波(高电平不一致)则输出上一次采样,或直接输出低电平
	//本值控制分频的倍数。
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	
	//向上计数说明从0开始计数到重装载值。
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	//ARR里头的值,ARR+1 = 预定值。 
	TIM_TimeBaseInitStructure.TIM_Period = 0XFFFF;
	
	//PSC分频。为0则不分频。
	TIM_TimeBaseInitStructure.TIM_Prescaler =0;
	
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter  = 0;
	
	//注意调用时基单元用的有指定TIM对象的这个。
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	//------配置编码器信号捕获。
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	//因为这个东西没初始化可能出现未定义错误之类因此我们需要用一下初始化函数
	TIM_ICStructInit(&TIM_ICInitStructure);
	
	//PA0为TIM4_CH1。
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	
	//滤波器直接拉满
	TIM_ICInitStructure.TIM_ICFilter = 0X0F;
	
	//配置上升沿捕获。
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	
	TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	//配置交叉捕获。
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	
	//滤波器直接拉满
	TIM_ICInitStructure.TIM_ICFilter = 0X0F;
	
	//配置上升沿捕获。
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;

	TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	//这东西相当于进行CH1,CH2的极性选择。。和上面那个进行配置是同样效果。(编码器本身就需要双边检测的)
	//Rising就是1为高,Failing相当于0为高的反相操作(从函数无both选项也可看出如此原理。)
	
	//这个极性选择的意思是,TIM_ICPolarity_Rising表示本通道接收的信号就是预期的接法。
	//配置为TIM_ICPolarity_Falling)意思是当接线AB相反接时(外围硬件电路),自动让B通道逻辑上落后一个相位(所谓反相)
	//也就是这个是对硬件接错的补救配置。。看接线了。。
	TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Falling);
	
	//这东西是硬件因此不需要占用CPU中断资源hh
	TIM_Cmd(TIM4,ENABLE);
}

//读已配置的这个传感器I/O口。
uint8_t RotaryEncoder_CheckInput(uint8_t KEYNum)
{
	return GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0 << KEYNum);
}

int GetRotaryEncoder()
{
	int16_t t = (int16_t)TIM_GetCounter(TIM4);
	return t;
}

int GetEncoderSpeed()
{
	return EcSpeed;
}

//每1s采样一次旋转的次数
//void TIM4_IRQHandler()
//{
//	if(TIM_GetITStatus(TIM4,TIM_IT_Update) == SET)
//	{
//		//标志位手动清除。
//		EcSpeed = (int16_t)GetRotaryEncoder();
//		//然后每次对计数清零
//		TIM_SetCounter(TIM4,0);
//		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
//	}
//}
