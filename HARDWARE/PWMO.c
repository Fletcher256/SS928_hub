#include "stm32f10x.h"

#include "PWMO.h"

#define TIM2_SERVO_PSC 72

#define TIM2_SERVO_ARR 20000

void ServoPWM_Init()
{
	//配置PWM外部GPIO口。
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//需要推挽驱动外部电路了。
	//注意这里用的是复用推挽。因为我们定时器输出的CHx通道连接GPIO调用推挽需要断开CPU连接那个开关,使用这个片上外设驱动。
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//用的内部时钟,挂载总线为1,计时器为2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
		
	//这个计时器内部默认用的内部时钟,不配也行。这个TIM2是APB1地址的偏移产物。
	TIM_InternalClockConfig(TIM2);
	
	//------------以下配置定时器的时基模块
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	//这个时钟分频是外部滤波器的采样频率,它可以直接由内部的时钟控制,频率越小,采样越慢,滤波效果越好。若发现杂波(高电平不一致)则输出上一次采样,或直接输出低电平
	//本值控制分频的倍数。
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	
	//向上计数说明从0开始计数到重装载值。
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	//ARR里头的值,ARR+1 = 预定值。 
	//驱动舵机的话记得把分辨率给高一点。(这里用20000,保证精度了)
	TIM_TimeBaseInitStructure.TIM_Period = TIM2_SERVO_ARR-1;
	
	//PSC分频。(这个是没有开放访问那个计数器的值的,GetPrescaler只能显示那个预分频常量。。)
	TIM_TimeBaseInitStructure.TIM_Prescaler =TIM2_SERVO_PSC -1;
	
	//如此配置则为1ms进一次中断。
	
	//高级计时器的重复计数功能。这个算是级联扩展计数范围用的,这里就不用配置了。
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter  = 0;
	
	//注意调用时基单元用的有指定TIM对象的这个。
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	//因为时基初始化结束后需要再次强制更新事件来将重装载值更新(有PSCbuffer是这样的)。这会导致CPU直接进入中断。因此下面把那个标志位给0先不让那个内部if执行
	//这样就不会每次初始化多执行一次中断了。
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	
	//------------以下配置OC口的控制寄存器
	
	TIM_OCInitTypeDef TIM_OCInitStructure;

	//如果用的通用定时器不需要那么多其他的功能,但是高级定时器的参数需要先初始化防止后续出现未定义行为。
	TIM_OCStructInit(&TIM_OCInitStructure);
	
	//这个比较器本质是控制计时到一定值,向外给出有效电平或无效电平,用这个外设结合GPIO实现对外部电路的自动控制而不占用CPU资源(不用反复进入中断来驱动PWM)
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//	
//	TIM_OCInitStructure.TIM_OCIdleState;
//	
//	TIM_OCInitStructure.TIM_OCNIdleState;
	
	//配置极性不翻转。(既有效电平为高。若有效电平为低算是低电平为高,极性翻转了。)	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//	
//	TIM_OCInitStructure.TIM_OCNPolarity;
//	
//	TIM_OCInitStructure.TIM_OutputNState;

	//输出使能
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	
	//pulse是输出脉冲控制,说白了就是配CCR。
	TIM_OCInitStructure.TIM_Pulse =0;
	
	//这里TIM2的CH1口是硬编码的,要么AFIO映射其他口(表中言之可用的口),要么就没辙。。
	//用4个PWM口来控制舵机进行转向。
	//TIM_OC1Init(TIM2,&TIM_OCInitStructure);
	TIM_OC2Init(TIM2,&TIM_OCInitStructure);
	//TIM_OC3Init(TIM2,&TIM_OCInitStructure);
	//TIM_OC4Init(TIM2,&TIM_OCInitStructure);
	
	//定时器触发器使能
	
	TIM_Cmd(TIM2,ENABLE);
}

//设置TIM2各路比较器的值
void SetTIM2CH1ARR(uint16_t t)
{
	TIM_SetCompare1(TIM2,t);
}

void SetTIM2CH2ARR(uint16_t t)
{
	TIM_SetCompare2(TIM2,t);
}

void SetTIM2CH3ARR(uint16_t t)
{
	TIM_SetCompare3(TIM2,t);
}

void SetTIM2CH4ARR(uint16_t t)
{
	TIM_SetCompare4(TIM2,t);
}

void SetServoRotation(float Angle)
{
	//实际测试的时候我们可以发现,若以小车的中线为90度的话,偏移度在-60~+60度之间。
	TIM_SetCompare2(TIM2,(Angle /180)*2000+500);
}

