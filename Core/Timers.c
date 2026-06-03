#include "stm32f10x.h"

//静态全局变量只有本文件内部可见

#define TIM1_KEY_PSC  100

#define TIM1_KEY_ARR  720

#define TIM4_REGULAR_PSC 100

#define TIM4_REGULAR_ARR 720

//系统时钟每1ms进一次中断。
#define SYSTEM_TICK 72000

static uint16_t Timer2_Cnt = 0;

static uint16_t Timer3_Cnt = 0;

void Timer1_Init()
{
	//用的外部时钟,挂载总线为1,计时器为2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
		
	//这个计时器内部默认用的内部时钟,不配也行。这个TIM2是APB1地址的偏移产物。
	TIM_InternalClockConfig(TIM1);
	
	//------------以下配置定时器的计时模块
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	//这个时钟分频是外部滤波器的采样频率,它可以直接由内部的时钟控制,频率越小,采样越慢,滤波效果越好。若发现杂波(高电平不一致)则输出上一次采样,或直接输出低电平
	//本值控制分频的倍数。
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;
	
	//向上计数说明从0开始计数到重装载值。
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	//ARR里头的值,ARR+1 = 预定值。 
	TIM_TimeBaseInitStructure.TIM_Period = TIM1_KEY_ARR-1;
	
	//PSC分频。(这个是没有开放访问那个计数器的值的,GetPrescaler只能显示那个预分频常量。。)
	TIM_TimeBaseInitStructure.TIM_Prescaler =TIM1_KEY_PSC-1;
	
	//如此配置则为1ms进一次中断。
	
	//高级计时器的重复计数功能。这个算是级联扩展计数范围用的,这里就不用配置了。
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter  = 0;
	
	//注意调用时基单元用的有指定TIM对象的这个。
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
	
	//因为时基初始化结束后需要再次强制更新事件来将重装载值更新(有PSCbuffer是这样的)。这会导致CPU直接进入中断。因此下面把那个标志位给0先不让那个内部if执行
	//这样就不会每次初始化多执行一次中断了。
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);

	//使能更新中断。
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
	
	//配置NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//用的TIM1的更新中断
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	
	NVIC_Init(&NVIC_InitStructure);
	
	//定时器触发器使能
	
	TIM_Cmd(TIM1,ENABLE);
}

void Timer2_Init()
{
	//用的外部时钟,挂载总线为1,计时器为2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
		
	//这个计时器内部默认用的内部时钟,不配也行。这个TIM2是APB1地址的偏移产物。
	TIM_InternalClockConfig(TIM2);
	
	//------------以下配置定时器的计时模块
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	//这个时钟分频是外部滤波器的采样频率,它可以直接由内部的时钟控制,频率越小,采样越慢,滤波效果越好。若发现杂波(高电平不一致)则输出上一次采样,或直接输出低电平
	//本值控制分频的倍数。
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;
	
	//向上计数说明从0开始计数到重装载值。
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	//ARR里头的值,ARR+1 = 预定值。 
	TIM_TimeBaseInitStructure.TIM_Period = 7199;
	
	//PSC分频。(这个是没有开放访问那个计数器的值的,GetPrescaler只能显示那个预分频常量。。)
	TIM_TimeBaseInitStructure.TIM_Prescaler =9999;
	
	//如此配置则为1ms进一次中断。
	
	//高级计时器的重复计数功能。这个算是级联扩展计数范围用的,这里就不用配置了。
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter  = 0;
	
	//注意调用时基单元用的有指定TIM对象的这个。
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	//因为时基初始化结束后需要再次强制更新事件来将重装载值更新(有PSCbuffer是这样的)。这会导致CPU直接进入中断。因此下面把那个标志位给0先不让那个内部if执行
	//这样就不会每次初始化多执行一次中断了。
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);

	//使能更新中断。
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
	//配置NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	
	NVIC_Init(&NVIC_InitStructure);
	
	//定时器触发器使能
	
	TIM_Cmd(TIM2,ENABLE);
}

void Timer4_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
		
	//这个计时器内部默认用的内部时钟,不配也行。这个TIM2是APB1地址的偏移产物。
	TIM_InternalClockConfig(TIM4);
	
	//------------以下配置定时器的计时模块
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	//这个时钟分频是外部滤波器的采样频率,它可以直接由内部的时钟控制,频率越小,采样越慢,滤波效果越好。若发现杂波(高电平不一致)则输出上一次采样,或直接输出低电平
	//本值控制分频的倍数。
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;
	
	//向上计数说明从0开始计数到重装载值。
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	//ARR里头的值,ARR+1 = 预定值。 
	TIM_TimeBaseInitStructure.TIM_Period = TIM4_REGULAR_ARR-1;
	
	//PSC分频。(这个是没有开放访问那个计数器的值的,GetPrescaler只能显示那个预分频常量。。)
	TIM_TimeBaseInitStructure.TIM_Prescaler =TIM4_REGULAR_PSC-1;
	
	//如此配置则为1ms进一次中断。
	
	//高级计时器的重复计数功能。这个算是级联扩展计数范围用的,这里就不用配置了。
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter  = 0;
	
	//注意调用时基单元用的有指定TIM对象的这个。
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	//因为时基初始化结束后需要再次强制更新事件来将重装载值更新(有PSCbuffer是这样的)。这会导致CPU直接进入中断。因此下面把那个标志位给0先不让那个内部if执行
	//这样就不会每次初始化多执行一次中断了。
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);

	//使能更新中断。
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	
	//配置NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//用的TIM4的更新中断
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	
	NVIC_Init(&NVIC_InitStructure);
	
	//定时器触发器使能
	
	TIM_Cmd(TIM4,ENABLE);
}

void ETR1_Init()
{
	//用的外部时钟,挂载总线为1,计时器为2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
		
	//这个计时器内部默认用的内部时钟,不配也行。这个TIM2是APB1地址的偏移产物。
	TIM_InternalClockConfig(TIM1);
	
	GPIO_InitTypeDef InitGPIOA;
	
	//配置外部触发ETR用的口。
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	//配上拉比较稳定。浮空的话除非外部的的电平很小会受内设影响,可以考虑用。,否则一般上下拉够用。
	InitGPIOA.GPIO_Mode = GPIO_Mode_IN_FLOATING;

	//这里外部输入时钟的ETR似乎都是与外部GPIO绑定的,比如这个TIM1_ETR必须用PA12。另外两个口也是如此(各配一个ETR),这样导致这种ETR口可能不够用。。
	InitGPIOA.GPIO_Pin = GPIO_Pin_12;
	
	InitGPIOA.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA,&InitGPIOA);

	
	//配置这里的ETR为下降沿中断触发使能,不分频(用的72MHZ采样),最后一个是滤波配置,在手册14.4.3,有对应的采样个数与频率配置,这个值越大滤波效果越好采样越慢。
	TIM_ETRClockMode1Config(TIM1,TIM_ExtTRGPSC_OFF,TIM_ExtTRGPolarity_Inverted,0X0F);
	
	//------------以下配置定时器的计时模块
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	//用这个高性能总线,它的预分频改为2就能够正常读取ETR(跳变小很多了)
	//等于是它将ETR外部时钟的频率按没2次来进行采样计数。。倒是满足这个传感器的特性了。。
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV2 ;

	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	//这里采样快一点了。
	TIM_TimeBaseInitStructure.TIM_Period = 9;

	TIM_TimeBaseInitStructure.TIM_Prescaler =0;

	TIM_TimeBaseInitStructure.TIM_RepetitionCounter  = 0;

	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);

	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);

	//使能更新中断。
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
	
	//配置NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	
	NVIC_Init(&NVIC_InitStructure);
	
	//定时器触发器使能
	
	TIM_Cmd(TIM1,ENABLE);
}

void SysTick_Init()
{
	    // 1. 设置重装载值（最大 24 位：0xFFFFFF）
    SysTick->LOAD = SYSTEM_TICK - 1;

    // 2. 清空当前计数器
    SysTick->VAL = 0;

    // 3. 配置控制寄存器：
    //    CLKSOURCE = 1   -> 使用 AHB 时钟（不分频）
    //    TICKINT   = 1   -> 使能中断
    //    ENABLE    = 1   -> 启动计数器
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;

    // 4. 设置中断优先级（可选，默认最低）
    NVIC_SetPriority(SysTick_IRQn, 0x0F);  
}

uint16_t GetCounter2()
{
	return Timer2_Cnt;
}

uint16_t GetCounter3()
{
	return Timer3_Cnt;
}

//void SysTick_Handler(void)
//{
//    // 每 1 ms 执行一次
//    sysTick_counter++;

//    // 你的周期性任务（尽量短小）
//    // 例如：读取按键、更新 LED 状态等
//}

////中断设置每1ms进入一次。
//void TIM1_UP_IRQHandler()
//{
//if(TIM_GetITStatus(TIM1,TIM_IT_Update) == SET)
//{
//	//这里用的非阻塞式按键检测。
//	static uint8_t keyCnt = 0;
//	
//	//每20ms扫描一次。
//	if(EXCOUNT(keyCnt,20))
//	{
//		UOB_KeyFrame();
//	}
//	
//	//标志位手动清除。
////		Timer3_Cnt++;
//	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
//}
//}

//void TIM2_IRQHandler()
//{
//if(TIM_GetITStatus(TIM2,TIM_IT_Update) == SET)
//{
//	//标志位手动清除。
//	Timer2_Cnt++;
//	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
//}
//}
//void TIM4_IRQHandler()
//{
//if(TIM_GetITStatus(TIM4,TIM_IT_Update) == SET)
//{
//	//标志位手动清除。
//	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
//}
//}
