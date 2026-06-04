#include "stm32f10x.h"

#include "Motors.h"

#include "Timers.h"

#include <math.h>

#define ACC 0.3

#define DELTA_T 80

#define ROT_NUM 2496

#define ROT_OFFSET 5

//小车轮胎直径:6.7cm
#define WHEEL_C 21.04867

#define ODOM_PI 3.1415926f

int16_t SpeedRank = 0;

int16_t is_Switch = 0;

PID_t rSpeed_PID = 
{
	.Target = 0,
	
	.Actual = 0,
	
	.Out = 0,
	
	.Kp = KP,
	
	.Ki = KI,
	
	.Kd = KD,

	.OutMax = 720,

	.OutMin = -720
};

PID_t lSpeed_PID = 
{
	.Target = 0,
	
	.Actual = 0,
	
	.Out = 0,
	
	.Kp = KP,
	
	.Ki = KI,
	
	.Kd = KD,

	.OutMax = 720,

	.OutMin = -720
};

PID_t Turn_PID = 
{
	.Target = 0,
	
	.Actual = 0,
	
	.Out = 0,
	
	.Kp = KP,
	
	.Ki = KI,
	
	.Kd = KD,

	.OutMax = 0,

	.OutMin = 0
};

SPEED_t rSpeed = 
{
	.Speed = 0,
	
	.tSpeed = 0,
	
	.SpeedCnt = 0
};

SPEED_t lSpeed = 
{
	.Speed = 0,
	
	.tSpeed = 0,
	
	.SpeedCnt = 0
};

PID_t Diff_PID =
{
	.Target = 0,

	.Actual = 0,

	.Out = 0,

	.Kp = KP,

	.Ki = KI,

	.Kd = KD,

	.OutMax = 720,

	.OutMin = -720
};

// ========== 舵机航向PID实例 (可通过串口在线修改Kp/Ki/Kd) ==========
HeadingPID_t headingPID =
{
	.Kp           = 2.5f,
	.Ki           = 0.01f,
	.Kd           = 0.18f,
	.MaxI         = 5.0f,
	.MaxOut       = 8.0f,
	.Deadband     = 2.0f,
	.D_Alpha      = 0.7f,
	.SmoothAlpha  = 0.4f,      // 输出平滑系数: 0.4新值+0.6旧值
	.CrossTrackKp     = 2.0f,
	.CrossTrackEnable = 1,     // 默认开启横向修正

	.Integral      = 0.0f,
	.LastError     = 0.0f,
	.dV            = 0.0f,
	.SmoothedAngle = 90.0f,
	.FirstRun      = 1,
};

float aveSpeed = 0;

float diffSpeed = 0;

// ========== 里程计 ==========
Odometry_t odom = {0};

void Odometry_Reset(void)
{
	odom.x = 0.0f;
	odom.y = 0.0f;
	odom.theta = 0.0f;
	odom.distance = 0.0f;
}

/*
 * 里程计更新(每20ms调用一次,与速度PID同步)
 * left_speed_cms:  左轮速度(cm/s), 正值=前进
 * right_speed_cms: 右轮速度(cm/s), 正值=前进
 * 基于两后轮差分推算航向变化和位移,不受IMU漂移影响
 */
void Odometry_Update(float left_speed_cms, float right_speed_cms)
{
	float dl = left_speed_cms * 0.02f;   // 20ms内左轮行驶距离(cm)
	float dr = right_speed_cms * 0.02f;  // 20ms内右轮行驶距离(cm)
	float dc = (dl + dr) * 0.5f;          // 中心点行驶距离

	// 编码器差分推算航向变化: 右轮多走 → 左转 → 航向角增加
	float raw_dtheta = (dr - dl) / WHEEL_TRACK_CM;

	// 方向修正: 后退时轮速为负,差分符号需翻转
	//  前进左转: dl=5,dr=7 → raw_dtheta>0 (正确,CCW)
	//  后退左转: dl=-5,dr=-7 → raw_dtheta<0 (错误,需翻转)
	float dtheta;
	if(dc >= 0.0f) {
		dtheta = raw_dtheta;
	} else {
		dtheta = -raw_dtheta;
	}

	// 中值积分提高精度
	float mid_theta = odom.theta + dtheta * 0.5f;
	// x is lateral (right positive), y is forward.
	odom.x -= dc * sinf(mid_theta);
	odom.y += dc * cosf(mid_theta);
	odom.theta += dtheta;
	while(odom.theta > ODOM_PI) odom.theta -= 2.0f * ODOM_PI;
	while(odom.theta < -ODOM_PI) odom.theta += 2.0f * ODOM_PI;
	odom.distance += fabsf(dc);
}

void PID_Init(PID_t *p)
{
	p->Target = 0;
	p->Actual = 0;
	p->Out = 0;
	p->Error0 = 0;
	p->Error1 = 0;
	p->ErrorInt = 0;
	p->pV = 0;
	p->iV = 0;
	p->dV = 0;
}

void Speed_Init(SPEED_t *sp)
{
	sp->Speed = 0;
	sp->tSpeed = 0;
	sp->SpeedCnt = 0;
}

void InitAll()
{
	PID_Init(&rSpeed_PID);

	PID_Init(&lSpeed_PID);

	PID_Init(&Turn_PID);

	Speed_Init(&rSpeed);

	Speed_Init(&lSpeed);
	
	aveSpeed = 0;

	diffSpeed = 0;
}

void Motor_Init()
{
	//配置PWM外部GPIO口。
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//注意。这里后续做反向运动的时候要修改PWM的接口状态的。
	
	//需要推挽驱动外部电路了。
	//注意这里用的是复用推挽。因为我们定时器输出的CHx通道连接GPIO调用推挽需要断开CPU连接那个开关,使用这个片上外设驱动。
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//用的内部时钟,挂载总线为1,计时器为2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
		
	//这个计时器内部默认用的内部时钟,不配也行。这个TIM1是APB1地址的偏移产物。
	TIM_InternalClockConfig(TIM1);
	
	//------------以下配置定时器的时基模块
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	//这个时钟分频是外部滤波器的采样频率,它可以直接由内部的时钟控制,频率越小,采样越慢,滤波效果越好。若发现杂波(高电平不一致)则输出上一次采样,或直接输出低电平
	//本值控制分频的倍数。
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	
	//向上计数说明从0开始计数到重装载值。
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	//ARR里头的值,ARR+1 = 预定值。 
	//驱动舵机的话记得把分辨率给高一点。(这里用20000,保证精度了)
	TIM_TimeBaseInitStructure.TIM_Period = TIM1_LMOTOR_ARR-1;
	
	//PSC分频。(这个是没有开放访问那个计数器的值的,GetPrescaler只能显示那个预分频常量。。)
	TIM_TimeBaseInitStructure.TIM_Prescaler =TIM1_LMOTOR_PSC -1;
	
	//如此配置则为1ms进一次中断。
	
	//高级计时器的重复计数功能。这个算是级联扩展计数范围用的,这里就不用配置了。
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter  = 0;
	
	//注意调用时基单元用的有指定TIM对象的这个。
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStructure);
	
	//因为时基初始化结束后需要再次强制更新事件来将重装载值更新(有PSCbuffer是这样的)。这会导致CPU直接进入中断。因此下面把那个标志位给0先不让那个内部if执行
	//这样就不会每次初始化多执行一次中断了。
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	
	//------------以下配置OC口的控制寄存器
	
	TIM_OCInitTypeDef TIM_OCInitStructure;

	//如果用的通用定时器不需要那么多其他的功能,但是高级定时器的参数需要先初始化防止后续出现未定义行为。
	TIM_OCStructInit(&TIM_OCInitStructure);
	
	//这里TIM1的CH1口是硬编码的,要么AFIO映射其他口(表中言之可用的口),要么就没辙。。
	//用4个PWM口来控制舵机进行转向。

		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		
		//配置极性不翻转。(既有效电平为高。若有效电平为低算是低电平为高,极性翻转了。)	
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

		//输出使能
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		
		//pulse是输出脉冲控制,说白了就是配CCR。
		TIM_OCInitStructure.TIM_Pulse =0;
		TIM_OC1Init(TIM1,&TIM_OCInitStructure);
		TIM_OC2Init(TIM1,&TIM_OCInitStructure);
		TIM_OC3Init(TIM1,&TIM_OCInitStructure);
		TIM_OC4Init(TIM1,&TIM_OCInitStructure);
		
		//开头默认正转
	//	SetLMRotate(2,0);
	
	//定时器触发器使能
	
	//高级定时器需要这个使能
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
	//TIM_ARRPreloadConfig(TIM1, ENABLE);
	
	TIM_Cmd(TIM1,ENABLE);
}

void MotorEnCoder_Init()
{
	GPIO_InitTypeDef InitGPIO;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);

	// 使用 SWD 调试接口，关闭 JTAG，释放 PB4 (JNTRST) 等引脚
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	// 第三步: 对 TIM3 通道1 进行部分重映射 (将功能从 PA6 重映射到 PB4,	PA7重映射到 PB5)
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
	
	InitGPIO.GPIO_Mode = GPIO_Mode_IPU;
	
	InitGPIO.GPIO_Pin = 0X00000000;

	//我们这里使用的是TIM4,TIM3的CH1与CH2
	InitGPIO.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_4 | GPIO_Pin_5;
	
	InitGPIO.GPIO_Speed = GPIO_Speed_50MHz;
	
	//初始化整个GPIO口。
	//修正这个说法。这里其实是初始化你选中这个GPIO组的这几个PIN口,不是全部扫描。
	GPIO_Init(GPIOB,&InitGPIO);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);	
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
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
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
	
	TIM_ICInit(TIM3,&TIM_ICInitStructure);
	
	//配置交叉捕获。
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	
	//滤波器直接拉满
	TIM_ICInitStructure.TIM_ICFilter = 0X0F;
	
	//配置上升沿捕获。
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;

	TIM_ICInit(TIM3,&TIM_ICInitStructure);
	
	TIM_ICInit(TIM4,&TIM_ICInitStructure);

	//这东西相当于进行CH1,CH2的极性选择。。和上面那个进行配置是同样效果。(编码器本身就需要双边检测的)
	//Rising就是1为高,Failing相当于0为高的反相操作(从函数无both选项也可看出如此原理。)
	
	//这个极性选择的意思是,TIM_ICPolarity_Rising表示本通道接收的信号就是预期的接法。
	//配置为TIM_ICPolarity_Falling)意思是当接线AB相反接时(外围硬件电路),自动让B通道逻辑上落后一个相位(所谓反相)
	//也就是这个是对硬件接错的补救配置。。看接线了。。
	TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Falling);
	
	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Falling);
	//这东西是硬件因此不需要占用CPU中断资源hh
	TIM_Cmd(TIM4,ENABLE);
	
	TIM_Cmd(TIM3,ENABLE);
}

void lSetSpeed(int16_t Speed)
{
	//反转
	if(Speed < 0)
	{
		TIM_SetCompare1(TIM1,0);
		TIM_SetCompare2(TIM1,-Speed);
	}
	else
	{
		TIM_SetCompare1(TIM1,Speed);
		TIM_SetCompare2(TIM1,0);
	}
}

void rSetSpeed(int16_t Speed)
{
	//反转
	if(Speed < 0)
	{
		TIM_SetCompare3(TIM1,0);
		TIM_SetCompare4(TIM1,-Speed);
	}
	else
	{
		TIM_SetCompare3(TIM1,Speed);
		TIM_SetCompare4(TIM1,0);
	}
}

int GetlEncoder()
{
	return (int16_t)TIM_GetCounter(TIM4);
}

int GetrEncoder()
{
	return (int16_t)TIM_GetCounter(TIM3);
}

void PID_Speed(PID_t *p,SPEED_t *sp,int8_t is_right)
{
	if(EXCOUNT(sp->SpeedCnt,20) == 1)
	{	
		p->Target = SpeedRank*RSPEEDSTEP/SPEEDSTEP;

		p->Actual = sp->tSpeed;

		p->Error0 = p->Target-p->Actual;
		
		p->pV = p->Kp * p->Error0;

		p->iV += p->Ki * p->Error0;

		if(p->iV<-120)
		{
			p->iV =-120;
		}
		if(p->iV>120)
		{
			p->iV = 120;
		}
		
		//积分限幅:当电机因为堵转或是停止供电之类,再次恢复供电时会导致积分项直接饱和,使电机进入饱和区间(会有一个饱和冲激,若断电/堵转过久会导致其反向积分回来要花较多时间造成电机满转跑飞)
		//这里建议使用实际测量到的值来估计
					
		//积分限幅会导致PWM下不去(超限了之后下不去。。)	

		//这个是比较两次误差的值,为负值则对out进行削弱
		p->dV = (1-alpha)*(p->Error0-p->Error1)*p->Kd + alpha*p->dV;
			
		p->Out = 8*(p->pV+ p->iV+p->dV);

		//输出限幅
		if(p->Out<p->OutMin)
		{
			p->Out = p->OutMin;
		}

		if(p->Out>p->OutMax)
		{
			p->Out = p->OutMax;
		}

		//这里需要积分偏移来启动

		//		if(ABS(SpeedRank) == 120)
		//		{			
		//			lOut = ABSTRACT(SpeedRank)*360;
		//		}

		if(SpeedRank == 0 && ABS(sp->tSpeed) < 50)
		{
			p->iV = 0;
			p->Out = 0;
		}

		if(is_right)
		{
			rSetSpeed((int16_t)p->Out);
		}
		else
		{
			lSetSpeed((int16_t)p->Out);
		}

		sp->Speed = WHEEL_C*sp->tSpeed / (0.02 * ROT_NUM);

		sp->tSpeed  = 0;
			
			p->Error1 = p->Error0;		

			// 左轮PID执行完毕,左右轮速度均为最新→更新里程计
			if(!is_right) {
				Odometry_Update(lSpeed.Speed, rSpeed.Speed);
			}
	}	
	sp->tSpeed += (is_right == 1 ? GetrEncoder() : GetlEncoder());

	is_right ? TIM_SetCounter(TIM3,0) : TIM_SetCounter(TIM4,0);
}

// ========== 舵机航向PID 初始化/复位 ==========

// 初始化为默认PID参数值
void HeadingPID_Init(HeadingPID_t *p)
{
	p->Kp           = 2.5f;
	p->Ki           = 0.01f;
	p->Kd           = 0.18f;
	p->MaxI         = 5.0f;
	p->MaxOut       = 8.0f;
	p->Deadband     = 0.9f;
	p->D_Alpha      = 0.7f;
	p->SmoothAlpha  = 0.4f;
	p->CrossTrackKp     = 2.0f;
	p->CrossTrackEnable = 1;

	p->Integral      = 0.0f;
	p->LastError     = 0.0f;
	p->dV            = 0.0f;
	p->SmoothedAngle = 90.0f;
	p->FirstRun      = 1;
}

// 复位内部状态(保留当前Kp/Ki/Kd设置,用于换向/重设直线时调用)
void HeadingPID_Reset(HeadingPID_t *p)
{
	p->Integral         = 0.0f;
	p->LastError        = 0.0f;
	p->dV               = 0.0f;
	p->SmoothedAngle    = 90.0f;
	p->FirstRun         = 1;
	p->CrossTrackEnable = 1;
}
