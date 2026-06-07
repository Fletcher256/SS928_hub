#include "stm32f10x.h"

#include "PWMO.h"

#define TIM2_SERVO_PSC 72

#define TIM2_SERVO_ARR 20000

#define SERVO_MIN_PULSE_US 1000U
#define SERVO_MAX_PULSE_US 2000U
#define SERVO_UPDATE_DEADBAND_DEG 0.8f

static uint16_t ServoLastPulseUs = 1500U;
static float ServoLastAngle = 90.0f;
static uint8_t ServoPWMInitialized = 0;
static uint16_t ServoRecoverCount = 0;

static uint16_t ServoAngleToPulse(float Angle)
{
	if(Angle < 0.0f)
	{
		Angle = 0.0f;
	}
	else if(Angle > 180.0f)
	{
		Angle = 180.0f;
	}
	return (uint16_t)((Angle / 180.0f) * (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) + (float)SERVO_MIN_PULSE_US);
}

uint8_t ServoPWM_IsHealthy(void)
{
	if(!ServoPWMInitialized)
	{
		return 0;
	}
	if(TIM2->PSC != (TIM2_SERVO_PSC - 1U))
	{
		return 0;
	}
	if(TIM2->ARR != (TIM2_SERVO_ARR - 1U))
	{
		return 0;
	}
	if((TIM2->CR1 & TIM_CR1_CEN) == 0U)
	{
		return 0;
	}
	if((TIM2->CCER & TIM_CCER_CC2E) == 0U)
	{
		return 0;
	}
	if(((GPIOA->CRL >> 4U) & 0x0FU) != 0x0BU)
	{
		return 0;
	}
	if((TIM2->CCR2 < SERVO_MIN_PULSE_US) || (TIM2->CCR2 > SERVO_MAX_PULSE_US))
	{
		return 0;
	}
	return 1;
}

static void ServoPWM_RecoverIfNeeded(void)
{
	if(!ServoPWM_IsHealthy())
	{
		ServoPWM_Init();
		ServoRecoverCount++;
	}
}

void ServoPWM_Service(void)
{
	if(!ServoPWM_IsHealthy())
	{
		ServoPWM_RecoverIfNeeded();
	}
	else
	{
		TIM_SetCompare2(TIM2, ServoLastPulseUs);
	}
}

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
	TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
	
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
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	//TIM_OC3Init(TIM2,&TIM_OCInitStructure);
	//TIM_OC4Init(TIM2,&TIM_OCInitStructure);
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	
	//定时器触发器使能
	
	TIM_Cmd(TIM2,ENABLE);
	TIM_SetCompare2(TIM2, ServoLastPulseUs);
	TIM_GenerateEvent(TIM2, TIM_EventSource_Update);
	ServoPWMInitialized = 1;
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
	uint8_t pwmHealthy;

	if(Angle < 0.0f)
	{
		Angle = 0.0f;
	}
	else if(Angle > 180.0f)
	{
		Angle = 180.0f;
	}
	//实际测试的时候我们可以发现,若以小车的中线为90度的话,偏移度在-60~+60度之间。
	pwmHealthy = ServoPWM_IsHealthy();
	if(pwmHealthy)
	{
		float diff = Angle - ServoLastAngle;
		if(diff < 0.0f)
		{
			diff = -diff;
		}
		if(diff < SERVO_UPDATE_DEADBAND_DEG)
		{
			return;
		}
	}
	ServoLastAngle = Angle;
	ServoLastPulseUs = ServoAngleToPulse(Angle);
	if(!pwmHealthy)
	{
		ServoPWM_RecoverIfNeeded();
	}
	TIM_SetCompare2(TIM2, ServoLastPulseUs);
}

float ServoPWM_GetLastAngle(void)
{
	return ServoLastAngle;
}

uint16_t ServoPWM_GetPulseUs(void)
{
	return ServoLastPulseUs;
}

uint16_t ServoPWM_GetPsc(void)
{
	return (uint16_t)TIM2->PSC;
}

uint16_t ServoPWM_GetArr(void)
{
	return (uint16_t)TIM2->ARR;
}

uint16_t ServoPWM_GetCcr2(void)
{
	return (uint16_t)TIM2->CCR2;
}

uint16_t ServoPWM_GetCcer(void)
{
	return (uint16_t)TIM2->CCER;
}

uint16_t ServoPWM_GetRecoverCount(void)
{
	return ServoRecoverCount;
}

