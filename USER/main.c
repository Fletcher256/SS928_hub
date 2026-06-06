#include "stm32f10x.h"

#include "string.h"

#include "Timers.h"

#include "Motors.h"

#include "PWMO.h"

//#include "OLED.h"

#include "USART.h"

#include "MPU6050.h"

#include <math.h>

#include "filter.h"

#include "LED.h"

#include "FSM.h"
#include "Steering.h"

// ========== 航向PID控制器 ==========
// 坐标约定: 舵机>90°=左转, <90°=右转; MPU6050 yaw: 左转为正,右转为负
// 调用周期: 与SysTick同步,当前每10ms执行一次(EXCOUNT(StraightCnt,10))
// PID参数已封装在 motors.c 的 headingPID 结构体中, 支持串口 ST_KP/ST_KI/ST_KD 在线修改
#define GYRO_FF_GAIN     0.25f   // GyroZ角速度前馈增益 (度/(度/秒))

//驱动板:只有D,C路是可以正常使用的,A,B路带负载能力极低,满转15Speed,且起转之后无法停止。

//注意正向的速度为负值(线序与驱动方向导致。)

//电机减速比为1:48,rpm = 220,反馈线数为13.

// ========== 与 FSM 共享的全局变量 ==========
int8_t is_up = 1;

int8_t is_turn = 0;

int8_t is_straight = 0;

//这个OLED是4*16的显示屏。

float Angle = 0;

MPU6050 MM;

float New_Yaw = 0;

float New_Roll = 0;

float New_Pitch = 0;

float Org_Yaw = 0;

KalmanFilter Kal_Yaw;
KalmanFilter Kal_Roll;
KalmanFilter Kal_Pitch;

volatile uint8_t TelemetryReady = 0;
volatile uint32_t ControlTicks = 0;

// ========== 航向保持 ==========

void Set_Straight()
{
	is_straight = 1;
	is_turn = 0;
	Org_Yaw = New_Yaw; // 记录当前(卡尔曼滤波后)航向作为目标直线航向
	// 复位PID全部状态
	HeadingPID_Reset(&headingPID);  // 复位航向PID全部状态(保留Kp/Ki/Kd设置)
	// 复位里程计: 以当前位置为原点,前进方向为Y轴
	headingPID.CrossTrackEnable = 1;
	Odometry_Reset();
}

/*
	坐标约定:
	 - 舵机 > 90° = 左转, < 90° = 右转
	 - MPU6050 yaw: 左转为正(+), 右转为负(-)
	 - error = target - current:
	     error > 0 → 车右偏 → correction > 0 → servo > 90 → 左转修正 ✓
	     error < 0 → 车左偏 → correction < 0 → servo < 90 → 右转修正 ✓

	PID结构:
	 - P项: 比例控制,快速响应偏差
	 - I项: 积分分离+限幅,消除稳态误差,防止windup
	 - D项: 不完全微分+低通滤波,预测趋势抑制超调与抖动
	 - 输出: 低通平滑滤波,防止舵机高频抖动
*/
void keep_straight()
{
	float error = Org_Yaw - New_Yaw; // 偏差: 正=车右偏需左修, 负=车左偏需右修 (使用卡尔曼滤波后的航向)

	// 180度跳变处理
	if(error > 180.0f) error -= 360.0f;
	if(error < -180.0f) error += 360.0f;

	// 死区: 小误差不响应
	if(fabs(error) < headingPID.Deadband) {
		error = 0.0f;
		headingPID.Integral *= 0.90f;   // 死区内衰减积分, 防止残值滞留
	}

	// ========== P项: 比例控制 ==========
	float pOut = headingPID.Kp * error;

	// ========== I项: 积分控制 ==========
	// 积分分离: 仅在小偏差(<8°)时积分,大偏差时缓慢泄放,防止积分饱和
	if(fabs(error) < 8.0f && fabs(error) > 0.0f) {
		headingPID.Integral += headingPID.Ki * error;
	} else if(fabs(error) >= 8.0f) {
		headingPID.Integral *= 0.95f; // 大偏差时缓慢泄放
	}
	// 积分限幅
	if(headingPID.Integral > headingPID.MaxI) headingPID.Integral = headingPID.MaxI;
	if(headingPID.Integral < -headingPID.MaxI) headingPID.Integral = -headingPID.MaxI;
	float iOut = headingPID.Integral;

	// ========== D项: 不完全微分 + 低通滤波 ==========
	// 首次调用跳过D,避免last_error=0时的微分冲击
	if(headingPID.FirstRun) {
		headingPID.LastError = error;
		headingPID.FirstRun  = 0;
	}
	float dError = error - headingPID.LastError;
	// 与速度PID(dV)一致的滤波结构: alpha=0.7低通
	headingPID.dV = (1.0f - headingPID.D_Alpha) * dError * headingPID.Kd
	           + headingPID.D_Alpha * headingPID.dV;
	float dOut = headingPID.dV;
	headingPID.LastError = error;

	// ========== 合成输出 ==========
	float correction = pOut + iOut + dOut;

	// 方向感知: 后退时舵机对航向的影响与前进相反,需取反修正量
	// is_up=1(前进): servo>90→左转→yaw↑   |  is_up=-1(后退): servo>90→左转→后退时车头实际右转→yaw↓
	if(is_up == -1) {
		correction = -correction;
		// 同时反转积分方向,避免前进时积攒的I项在后退时推错方向
		// (在合成后翻转等价于对整体修正取反,保持PID内部状态不变)
	}

	// ========== 横向偏差控制(基于编码器里程计,不受IMU漂移影响) ==========
	// 航向PID只看角度,5°偏差跑3秒=~8cm横向偏移,PID感知不到
	// 里程计直接测量横向位移,补偿航向控制的盲区
	// odom.x>0=车偏右 → cross_correction>0 → servo>90 → 左转修正
	if(headingPID.CrossTrackEnable) {
		float cross_correction = headingPID.CrossTrackKp * odom.x;
		// 后退时横向修正方向也需翻转
		if(is_up == -1) {
			cross_correction = -cross_correction;
		}
		correction += cross_correction;
	}

	// 输出限幅
	if(correction > headingPID.MaxOut) correction = headingPID.MaxOut;
	if(correction < -headingPID.MaxOut) correction = -headingPID.MaxOut;

	// ========== 输出低通平滑(防舵机高频抖动) ==========
	float target_angle = 90.0f + correction;
	// alpha=0.4: 适度平滑,兼顾响应速度与抗抖
	headingPID.SmoothedAngle = headingPID.SmoothAlpha * target_angle + (1.0f - headingPID.SmoothAlpha) * headingPID.SmoothedAngle;

		Steering_SetAngle(headingPID.SmoothedAngle);


	// ========== GyroZ角速度前馈(即时感知旋转,零漂移) ==========
	// 不等yaw角度积累误差,直接用角速度瞬时值反打舵机抑制旋转趋势
	// GyroZ是直接测量值,不存在积分漂移,从根本上免疫漂移问题
	// 前馈方向: 车在左转(GyroZ>0) → 负修正 → 舵机右打 → 抑制左转
	// float gyro_dps = MM.GyroZ / 16.4f;             // 角速度 °/s
	// float ff_correction = -GYRO_FF_GAIN * gyro_dps; // 负反馈: 旋转反方向打舵
	// // 前馈修正直接叠加到舵机(绕过PID平滑,保证响应速度)
	// Angle += ff_correction;
	// // 限幅保护
	// if(Angle > 90.0f + headingPID.MaxOut) Angle = 90.0f + headingPID.MaxOut;
	// if(Angle < 90.0f - headingPID.MaxOut) Angle = 90.0f - headingPID.MaxOut;

}

// ========== 主循环 ==========

int main ()
{
	KalmanFilter_Init(&Kal_Yaw,0.04,0.2,0,100);   // q=0.04,r=0.2: 增益~17%, ~80ms跟踪航向, 同时平滑噪声   // q=0.5: 稳态增益~83%,快速跟踪yaw变化(原0.01太慢仅吸收9%)
	KalmanFilter_Init(&Kal_Roll,0.01,0.1,1,100);
	KalmanFilter_Init(&Kal_Pitch,0.01,0.1,1,100);
	LED_Init();
	USART3_Init();
	//MPU6050_Init();
	MPU6050_init(GPIOB, GPIO_Pin_0, GPIO_Pin_1);
	//也也许我们需要对MPU6050进行一个静态校准。
	//MPU6050_Calibration();
	//mpu_dmp_init(GPIOB,GPIO_Pin_1,GPIO_Pin_0);
	MotorEnCoder_Init();

	//开始使能给0,不能满足为1时间足够长因此无法输出。
	//所以AT4950也需要一个初始化,就是上电先把它唤醒。。。
	//OLED_Init();
	SysTick_Init();
	ServoPWM_Init();
		Steering_Center();

	Motor_Init();
	SetStandbyMode();
	RefreshCommandWatchdog();

	//校验MPU6050是否成功读到数据。
	USART3_printf("Everything is ready!\r\n");
	while(1)
	{
		//mpu_dmp_get_data(&MM.pitch,&MM.roll,&MM.yaw);
		//读取标志位就绪 → 统一命令处理入口
		if(GetUSART3RXTState() == 1)
		{
			HandleTextCommand(GetUSART3TextBuffer());
		}

		// 任务调度: 超时保护 + 自动驾驶步进 + 完成检测
		UpdateControlTask();

		//USART3_printf("%d,%d,%d,%d,%d,%d\r\n",MD.xAcc,MD.yAcc,MD.zAcc,MD.xGyro,MD.yGyro,MD.zGyro);

		//atan2:可以计算-180deg到180deg,第一个形参是分子。
		//USART3_printf("%f,%f,%f,%f,%f,%f,%f\r\n",EA.MPU6050_Yaw,EA.MPU6050_Roll,EA.MPU6050_Pitch,MD.zAcc*G*16/(0X7FFF),atan2(MD.xAcc,MD.yAcc)/PI*180,atan2(MD.yAcc,MD.zAcc)/PI*180,atan2(MD.xAcc,MD.zAcc)/PI*180);
		 //USART3_printf("%.3f,%.3f,%.3f\r\n", MM.roll, MM.pitch, MM.yaw);
		 //USART3_printf("%f,%f,%f,%d,%f,%f\r\n",rSpeed.Speed,lSpeed.Speed,aveSpeed,SpeedRank,rSpeed_PID.Out,lSpeed_PID.Out);
		 if(TelemetryReady)
		 {
			 TelemetryReady = 0;
			 USART3_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",MM.GyroX/16.4f, New_Yaw, Angle, aveSpeed, odom.x, odom.y, odom.theta, lSpeed.Speed, rSpeed.Speed);
		 }
	}
}

void SysTick_Handler(void)
{
	ControlTicks++;
	//GetALLData(&MD);
	//CalEulerAngleHandler(&MD);
	//ComplementaryFilter(&MD);

	static uint16_t SwitchCnt = 0;
	static uint16_t MPU6050Cnt = 0;
	static uint16_t StraightCnt = 0;
	static uint16_t TelemetryCnt = 0;

	if(EXCOUNT(MPU6050Cnt,5) == 1)
	{
		MPU6050_Get_Angle(&MM);

		//这里做一个读取丢包检测。如果一个数据超过8次没有任何变化那么认为MPU6050丢包,直接重新读取。

		New_Pitch = KalmanFilter_Update(&Kal_Pitch,MM.pitch);

		New_Roll = KalmanFilter_Update(&Kal_Roll,MM.roll);

		New_Yaw = KalmanFilter_Update(&Kal_Yaw,MM.yaw);
	}
	if(is_Switch)
	{
		//换向时停摆50ms
		if(EXCOUNT(SwitchCnt,20) == 1)
		{
			is_Switch = 0;
		}
	}
	else
	{
		//AccContrllor();
	//因为AT4950的特性(唤醒),所以我们针对它来对这个PID环进行改进
	//初始保持两路PWM不需要担心无法唤醒


		PID_Speed(&rSpeed_PID,&rSpeed,1);
		PID_Speed(&lSpeed_PID,&lSpeed,0);
		//这个会导致正常调电机没法正常转向,调舵机的时候注意先把它关掉啊。。。
		if(is_straight && (EXCOUNT(StraightCnt,20) == 1))
		{
			keep_straight() ;
		}

		aveSpeed = (rSpeed.Speed + lSpeed.Speed)*0.5f;
	}

	if(EXCOUNT(TelemetryCnt,100) == 1)
	{
		TelemetryReady = 1;
	}
}
