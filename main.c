#include "stm32f10x.h"
#include "Timers.h"
#include "Motors.h"
#include "PWMO.h"
//#include "OLED.h"
#include "USART.h"
#include "MPU6050.h"
#include "filter.h"
#include "LED.h"

#include "string.h"
#include <math.h>

// ========== 航向PID控制器 ==========
// 坐标约定: 舵机>90°=左转, <90°=右转; MPU6050 yaw: 左转为正,右转为负
// 调用周期: 与SysTick同步,当前每10ms执行一次(EXCOUNT(StraightCnt,10))
// PID参数已封装在 motors.c 的 headingPID 结构体中, 支持串口 ST_KP/ST_KI/ST_KD 在线修改
#define GYRO_FF_GAIN     0.25f   // GyroZ角速度前馈增益 (度/(度/秒))

//驱动板:只有D,C路是可以正常使用的,A,B路带负载能力极低,满转15Speed,且起转之后无法停止。

//注意正向的速度为负值(线序与驱动方向导致。)

//电机减速比为1:48,rpm = 220,反馈线数为13.

//换向标志位。
int8_t is_up = 1;

//以下为状态跳转位。
uint8_t is_Pause = 1;

int8_t is_turn = 0;

int8_t is_straight = 0;

typedef enum RunState
{
	STANDBY = 0,
	PARKING,
	HITTED
}RS;

RS rS = STANDBY;

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
static volatile uint16_t MpuTaskElapsedMs = 0;
static volatile uint8_t StraightTaskReady = 0;

#define REMOTE_TIMEOUT_MS       2000U
#define DISTANCE_TIMEOUT_MS     30000U
#define TURN_TIMEOUT_MS         8000U
#define AUTO_DEFAULT_SPEED      2U
#define AUTO_FORWARD1_CM        100.0f
#define AUTO_FORWARD2_CM        60.0f
#define AUTO_TURN_DEG           90.0f
#define DISTANCE_DONE_CM        2.0f
#define TURN_DONE_DEG           3.0f
#define TURN_SERVO_MAX_OFFSET   35.0f
#define TURN_SERVO_KP           0.75f

typedef enum ControlMode
{
	CTRL_IDLE = 0,
	CTRL_MANUAL,
	CTRL_STRAIGHT,
	CTRL_DISTANCE,
	CTRL_TURN_YAW,
	CTRL_AUTO_ROUTE
} ControlMode_t;

typedef enum AutoStep
{
	AUTO_IDLE = 0,
	AUTO_FORWARD1,
	AUTO_TURN1,
	AUTO_FORWARD2
} AutoStep_t;

static ControlMode_t ControlMode = CTRL_IDLE;
static AutoStep_t AutoStep = AUTO_IDLE;
static uint32_t LastCommandTick = 0;
static uint32_t ActionStartTick = 0;
static float TargetDistanceCm = 0.0f;
static float TargetYaw = 0.0f;
static uint8_t AutoSpeedLevel = AUTO_DEFAULT_SPEED;

void ExDirect(uint8_t Rot);
void SetSpeedRank(int8_t level);
void Set_Straight(void);

static uint8_t IsDigitChar(char c)
{
	return (c >= '0' && c <= '9');
}

static const char *SkipValuePrefix(const char *s)
{
	while(*s == ' ' || *s == '=' || *s == ':')
	{
		s++;
	}
	return s;
}

static uint8_t ParseCommandValue(const char *s, float *value, uint8_t scaledHundredths)
{
	float integer = 0.0f;
	float fraction = 0.0f;
	float scale = 1.0f;
	int8_t sign = 1;
	uint8_t hasDigit = 0;
	uint8_t hasDot = 0;

	s = SkipValuePrefix(s);

	if(*s == '-')
	{
		sign = -1;
		s++;
	}
	else if(*s == '+')
	{
		s++;
	}

	while(IsDigitChar(*s))
	{
		integer = integer * 10.0f + (float)(*s - '0');
		hasDigit = 1;
		s++;
	}

	if(*s == '.')
	{
		hasDot = 1;
		s++;
		while(IsDigitChar(*s))
		{
			scale *= 10.0f;
			fraction += (float)(*s - '0') / scale;
			hasDigit = 1;
			s++;
		}
	}

	s = SkipValuePrefix(s);
	if(!hasDigit || *s != '\0')
	{
		return 0;
	}

	*value = (integer + fraction) * (float)sign;
	if(scaledHundredths && !hasDot)
	{
		*value *= 0.01f;
	}
	return 1;
}

static float ClampFloat(float value, float minValue, float maxValue)
{
	if(value < minValue) return minValue;
	if(value > maxValue) return maxValue;
	return value;
}

static float AbsFloat(float value)
{
	return value < 0.0f ? -value : value;
}

static float NormalizeYaw(float yaw)
{
	while(yaw > 180.0f) yaw -= 360.0f;
	while(yaw < -180.0f) yaw += 360.0f;
	return yaw;
}

static float GetYawError(float target, float current)
{
	return NormalizeYaw(target - current);
}

static void RefreshCommandWatchdog(void)
{
	LastCommandTick = ControlTicks;
}

static void CenterSteering(void)
{
	Angle = 90.0f;
	SetServoRotation(Angle);
}

static void HardStopMotion(void)
{
	SpeedRank = 0;
	rSetSpeed(0);
	lSetSpeed(0);
	InitAll();
	CenterSteering();
	is_straight = 0;
	is_turn = 0;
	AutoStep = AUTO_IDLE;
	ControlMode = CTRL_IDLE;
}

static void SetStandbyMode(void)
{
	HardStopMotion();
	rS = STANDBY;
	SetLEDs(GPIO_Pin_14);
}

static void SetManualMode(void)
{
	if(ControlMode == CTRL_AUTO_ROUTE || ControlMode == CTRL_DISTANCE || ControlMode == CTRL_TURN_YAW)
	{
		AutoStep = AUTO_IDLE;
	}
	ControlMode = CTRL_MANUAL;
	is_straight = 0;
	is_turn = 0;
	headingPID.CrossTrackEnable = 0;
}

static void SetManualModeIfIdle(void)
{
	if(ControlMode == CTRL_IDLE)
	{
		SetManualMode();
	}
}

static void EnsureAutoSpeed(void)
{
	if(SpeedRank == 0)
	{
		SetSpeedRank(AutoSpeedLevel);
	}
}

static void PrepareStraightHold(void)
{
	Set_Straight();
	ControlMode = CTRL_STRAIGHT;
	AutoStep = AUTO_IDLE;
}

static uint8_t PrepareDistanceDrive(float distanceCm)
{
	if(AbsFloat(distanceCm) < 1.0f)
	{
		return 0;
	}

	if(distanceCm < 0.0f)
	{
		if(is_up == 1)
		{
			ExDirect(0);
		}
		TargetDistanceCm = -distanceCm;
	}
	else
	{
		if(is_up == -1)
		{
			ExDirect(1);
		}
		TargetDistanceCm = distanceCm;
	}

	Set_Straight();
	Odometry_Reset();
	ActionStartTick = ControlTicks;
	EnsureAutoSpeed();
	return 1;
}

static uint8_t StartDistanceDrive(float distanceCm)
{
	if(PrepareDistanceDrive(distanceCm))
	{
		ControlMode = CTRL_DISTANCE;
		AutoStep = AUTO_IDLE;
		USART3_printf("Distance drive %.1f cm\r\n", TargetDistanceCm);
		return 1;
	}
	else
	{
		USART3_printf("Invalid distance target!\r\n");
		return 0;
	}
}

static uint8_t PrepareYawTurn(float relativeYawDeg)
{
	if(AbsFloat(relativeYawDeg) < TURN_DONE_DEG)
	{
		return 0;
	}

	TargetYaw = NormalizeYaw(New_Yaw + relativeYawDeg);
	is_straight = 0;
	is_turn = 1;
	headingPID.CrossTrackEnable = 0;
	ActionStartTick = ControlTicks;
	EnsureAutoSpeed();
	return 1;
}

static uint8_t StartYawTurn(float relativeYawDeg)
{
	if(PrepareYawTurn(relativeYawDeg))
	{
		ControlMode = CTRL_TURN_YAW;
		AutoStep = AUTO_IDLE;
		USART3_printf("Yaw turn %.1f deg\r\n", relativeYawDeg);
		return 1;
	}
	else
	{
		USART3_printf("Invalid yaw target!\r\n");
		return 0;
	}
}

static uint8_t StartAutoRoute(void)
{
	rS = PARKING;
	SetLEDs(GPIO_Pin_12);
	AutoSpeedLevel = AUTO_DEFAULT_SPEED;
	ControlMode = CTRL_AUTO_ROUTE;
	AutoStep = AUTO_FORWARD1;
	if(PrepareDistanceDrive(AUTO_FORWARD1_CM))
	{
		USART3_printf("Auto route start\r\n");
		return 1;
	}
	else
	{
		SetStandbyMode();
		USART3_printf("Auto route failed\r\n");
		return 0;
	}
}

static uint8_t UpdateDistanceDrive(void)
{
	Odometry_t snapshot;

	if(TargetDistanceCm <= 0.0f)
	{
		return 1;
	}
	Odometry_GetSnapshot(&snapshot);
	if((TargetDistanceCm - snapshot.distance) <= DISTANCE_DONE_CM)
	{
		return 1;
	}
	return 0;
}

static uint8_t UpdateYawTurn(void)
{
	float error = GetYawError(TargetYaw, New_Yaw);
	float correction;

	if(AbsFloat(error) <= TURN_DONE_DEG)
	{
		SpeedRank = 0;
		CenterSteering();
		is_turn = 0;
		return 1;
	}

	correction = ClampFloat(error * TURN_SERVO_KP, -TURN_SERVO_MAX_OFFSET, TURN_SERVO_MAX_OFFSET);
	if(is_up == -1)
	{
		correction = -correction;
	}

	Angle = ClampFloat(90.0f + correction, 0.0f, 180.0f);
	SetServoRotation(Angle);
	EnsureAutoSpeed();
	return 0;
}

static void UpdateControlTask(void)
{
	if((ControlMode == CTRL_MANUAL || ControlMode == CTRL_STRAIGHT) &&
	   SpeedRank != 0 &&
	   (uint32_t)(ControlTicks - LastCommandTick) > REMOTE_TIMEOUT_MS)
	{
		SetStandbyMode();
		USART3_printf("Remote timeout stop!\r\n");
		return;
	}

	if((ControlMode == CTRL_DISTANCE ||
	   (ControlMode == CTRL_AUTO_ROUTE && (AutoStep == AUTO_FORWARD1 || AutoStep == AUTO_FORWARD2))) &&
	   (uint32_t)(ControlTicks - ActionStartTick) > DISTANCE_TIMEOUT_MS)
	{
		SetStandbyMode();
		USART3_printf("Distance timeout stop!\r\n");
		return;
	}

	if((ControlMode == CTRL_TURN_YAW ||
	   (ControlMode == CTRL_AUTO_ROUTE && AutoStep == AUTO_TURN1)) &&
	   (uint32_t)(ControlTicks - ActionStartTick) > TURN_TIMEOUT_MS)
	{
		SetStandbyMode();
		USART3_printf("Turn timeout stop!\r\n");
		return;
	}

	if(ControlMode == CTRL_DISTANCE)
	{
		if(UpdateDistanceDrive())
		{
			SetStandbyMode();
			USART3_printf("Distance done\r\n");
		}
	}
	else if(ControlMode == CTRL_TURN_YAW)
	{
		if(UpdateYawTurn())
		{
			SetStandbyMode();
			USART3_printf("Yaw turn done\r\n");
		}
	}
	else if(ControlMode == CTRL_AUTO_ROUTE)
	{
		if(AutoStep == AUTO_FORWARD1)
		{
			if(UpdateDistanceDrive())
			{
				SpeedRank = 0;
				if(PrepareYawTurn(AUTO_TURN_DEG))
				{
					AutoStep = AUTO_TURN1;
				}
				else
				{
					SetStandbyMode();
					USART3_printf("Auto turn failed\r\n");
				}
			}
		}
		else if(AutoStep == AUTO_TURN1)
		{
			if(UpdateYawTurn())
			{
				if(PrepareDistanceDrive(AUTO_FORWARD2_CM))
				{
					AutoStep = AUTO_FORWARD2;
				}
				else
				{
					SetStandbyMode();
					USART3_printf("Auto forward failed\r\n");
				}
			}
		}
		else if(AutoStep == AUTO_FORWARD2)
		{
			if(UpdateDistanceDrive())
			{
				SetStandbyMode();
				rS = PARKING;
				SetLEDs(GPIO_Pin_12);
				USART3_printf("Auto route done\r\n");
			}
		}
	}
}


void SpeedAcc()
{		
		int16_t rank = ABS(SpeedRank);
		if(rank < 720)
		{
			rank += SPEEDSTEP;
			if(rank > 720) rank = 720;
			SpeedRank = ABSTRACT(is_up) * rank;
		}
}

void SpeedSlowDown()
{		
		int16_t rank = ABS(SpeedRank);
		if(rank > 0)
		{
			rank -= SPEEDSTEP;
			if(rank < 0) rank = 0;
			SpeedRank = ABSTRACT(is_up) * rank;
		}
}

void ExDirect(uint8_t Rot)
{
	if(Rot)
	{
		is_up = 1;
			
		SpeedRank = ABS(SpeedRank);		
	}
	else
	{
		is_up = -1;
		SpeedRank = -ABS(SpeedRank);
	}
	InitAll();
	Org_Yaw = New_Yaw;  // 换向时同步更新目标航向(卡尔曼滤波后),避免PID从非零error冷启动→超调
	// 复位航向PID全部状态
	HeadingPID_Reset(&headingPID);  // 复位航向PID全部状态(保留Kp/Ki/Kd设置)
	Odometry_Reset();  // 换向时里程计重新标定原点
	if(is_straight)
	{
		headingPID.CrossTrackEnable = 1;
	}
	is_Switch =1;
}

//一共有6档,默认低速3档(平稳移动),456逐步加速
void SetSpeedRank(int8_t level)
{
	if(level < 7 && level > -1)
	{
		SpeedRank = is_up*level*SPEEDSTEP;
	}
}

void SoftReset(void)
{
    __disable_irq();       // 关闭普通中断
	for(int i = 0;i<10000;i++);
    NVIC_SystemReset();    // 触发系统复位
}

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
		Odometry_t snapshot;
		Odometry_GetSnapshot(&snapshot);
		float cross_correction = headingPID.CrossTrackKp * snapshot.x;
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

	Angle = headingPID.SmoothedAngle;
	
	//SetServoRotation(Angle);

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
	SetServoRotation(Angle);
}

static void HandleTextCommand(char *pBuffer)
{
	float value = 0.0f;
	uint8_t commandAccepted = 0;

	if(strcmp((const char *)pBuffer, "RC_HB") == 0)
	{
		USART3_printf("OK\r\n");
		commandAccepted = 1;
	}
	else if(strcmp((const char *)pBuffer, "RC_STOP") == 0 || strcmp((const char *)pBuffer, "AU_STOP") == 0)
	{
		USART3_printf("Stop!\r\n");
		SetStandbyMode();
		commandAccepted = 1;
	}
	else if(strcmp((const char *)pBuffer, "RC_MAN") == 0)
	{
		SetManualMode();
		USART3_printf("Manual mode\r\n");
		commandAccepted = 1;
	}
	else if(strcmp((const char *)pBuffer, "RC_STR") == 0)
	{
		PrepareStraightHold();
		USART3_printf("Straight hold mode\r\n");
		commandAccepted = 1;
	}
	else if(strcmp((const char *)pBuffer, "RC_AUTO") == 0 || strcmp((const char *)pBuffer, "AU_RUN") == 0)
	{
		commandAccepted = StartAutoRoute();
	}
	else if(strncmp((const char *)pBuffer, "RC_DST", 6) == 0)
	{
		if(ParseCommandValue(&pBuffer[6], &value, 0))
		{
			commandAccepted = StartDistanceDrive(value);
		}
		else
		{
			USART3_printf("Invalid distance value!\r\n");
		}
	}
	else if(strncmp((const char *)pBuffer, "RC_YAW", 6) == 0)
	{
		if(ParseCommandValue(&pBuffer[6], &value, 0))
		{
			commandAccepted = StartYawTurn(value);
		}
		else
		{
			USART3_printf("Invalid yaw value!\r\n");
		}
	}
	else if(strncmp((const char *)pBuffer, "RC_SPD", 6) == 0)
	{
		if(ParseCommandValue(&pBuffer[6], &value, 0) && value >= 0.0f && value <= 6.0f)
		{
			if(ControlMode == CTRL_AUTO_ROUTE || ControlMode == CTRL_DISTANCE || ControlMode == CTRL_TURN_YAW)
			{
				SetManualMode();
			}
			SetManualModeIfIdle();
			SetSpeedRank((int8_t)value);
			USART3_printf("SET %d Rank!\r\n", SpeedRank);
			commandAccepted = 1;
		}
		else
		{
			USART3_printf("Invalid speed rank!\r\n");
		}
	}
	else if(strncmp((const char *)pBuffer, "RC_STE", 6) == 0)
	{
		if(ParseCommandValue(&pBuffer[6], &value, 0))
		{
			SetManualMode();
			Angle = ClampFloat(value, 0.0f, 180.0f);
			SetServoRotation(Angle);
			USART3_printf("Servo to %f deg!\r\n", Angle);
			commandAccepted = 1;
		}
		else
		{
			USART3_printf("Invalid servo value!\r\n");
		}
	}
	else if(strcmp((const char *)pBuffer,COMMANDS[7]) == 0)
	{
		USART3_printf("Reset!\r\n");
		commandAccepted = 1;
		SoftReset();
	}
	else if(strcmp((const char *)pBuffer,COMMANDS[6]) == 0)
	{
		SetManualMode();
		USART3_printf("Down!\r\n");
		if(is_up == 1)ExDirect(0);
		commandAccepted = 1;
	}
	else if(strcmp((const char *)pBuffer,COMMANDS[5]) == 0)
	{
		SetManualMode();
		USART3_printf("Up!\r\n");
		if(is_up == -1)ExDirect(1);
		commandAccepted = 1;
	}
	else if(strcmp((const char *)pBuffer,COMMANDS[12]) == 0)
	{
		USART3_printf("Stand by!\r\n");
		SetStandbyMode();
		commandAccepted = 1;
	}
	else if(strcmp((const char *)pBuffer,COMMANDS[10]) == 0)
	{
		USART3_printf("Parking auto!\r\n");
		commandAccepted = StartAutoRoute();
	}
	else if(strcmp((const char *)pBuffer,COMMANDS[11]) == 0)
	{
		USART3_printf("Hitted!\r\n");
		SetStandbyMode();
		rS = HITTED;
		SetLEDs(GPIO_Pin_13);
		commandAccepted = 1;
	}
	else if(strcmp((const char *)pBuffer,COMMANDS[8]) == 0)
	{
		USART3_printf("Straight!\r\n");
		PrepareStraightHold();
		commandAccepted = 1;
	}
	else if(strcmp((const char *)pBuffer,COMMANDS[9]) == 0)
	{
		SetManualMode();
		is_turn = 1;
		USART3_printf("Turn manual mode!\r\n");
		commandAccepted = 1;
	}
	else if(strcmp((const char *)pBuffer,COMMANDS[0]) == 0)
	{
		if(ControlMode == CTRL_AUTO_ROUTE || ControlMode == CTRL_DISTANCE || ControlMode == CTRL_TURN_YAW)
		{
			SetManualMode();
		}
		SetManualModeIfIdle();
		USART3_printf("SpeedRank add!\r\n");
		SpeedAcc();
		commandAccepted = 1;
	}
	else if(strcmp((const char *)pBuffer,COMMANDS[1]) == 0)
	{
		if(ControlMode == CTRL_AUTO_ROUTE || ControlMode == CTRL_DISTANCE || ControlMode == CTRL_TURN_YAW)
		{
			SetManualMode();
		}
		SetManualModeIfIdle();
		USART3_printf("SpeedRank decline!\r\n");
		SpeedSlowDown();
		commandAccepted = 1;
	}
	else if(strcmp((const char *)pBuffer,COMMANDS[3]) == 0)
	{
		SetManualMode();
		USART3_printf("SpeedRank stop!\r\n");
		SpeedRank = 0;
		rSetSpeed(0);
		lSetSpeed(0);
		commandAccepted = 1;
	}
	else if(strncmp((const char *)pBuffer,COMMANDS[13],4) == 0)
	{
		float t = 0.0f;
		if((pBuffer[4] == 'P' || pBuffer[4] == 'I' || pBuffer[4] == 'D') &&
		   ParseCommandValue(&pBuffer[5], &t, 1))
		{
			switch(pBuffer[4])
			{
			case 'P':
				headingPID.Kp = t;
				USART3_printf("Set heading Kp to %f!\r\n",headingPID.Kp);
				commandAccepted = 1;
				break;
			case 'I':
				headingPID.Ki = t;
				USART3_printf("Set heading Ki to %f!\r\n",headingPID.Ki);
				commandAccepted = 1;
				break;
			case 'D':
				headingPID.Kd = t;
				USART3_printf("Set heading Kd to %f!\r\n",headingPID.Kd);
				commandAccepted = 1;
				break;
			default:
				USART3_printf("Unknown heading PID command!\r\n");
				break;
			}
		}
		else
		{
			USART3_printf("Invalid heading PID value!\r\n");
		}
	}
	else if(strncmp((const char *)pBuffer,COMMANDS[4],5) == 0)
	{
		float rotateCmd = 0.0f;
		if(ParseCommandValue(&pBuffer[5], &rotateCmd, 0))
		{
			SetManualMode();
			Angle = ClampFloat(180.0f - rotateCmd, 0.0f, 180.0f);
			commandAccepted = 1;
		}
		else
		{
			USART3_printf("Invalid rotate value!\r\n");
			return;
		}
		USART3_printf("Rotate to %f deg!\r\n",Angle);
		SetServoRotation(Angle);
	}
	else if(strncmp((const char *)pBuffer,COMMANDS[2],6) == 0)
	{
		if(pBuffer[6] >= '0' && pBuffer[6] <= '6')
		{
			if(ControlMode == CTRL_AUTO_ROUTE || ControlMode == CTRL_DISTANCE || ControlMode == CTRL_TURN_YAW)
			{
				SetManualMode();
			}
			SetManualModeIfIdle();
			SetSpeedRank(pBuffer[6]-'0');
			USART3_printf("SET %d Rank!\r\n",SpeedRank);
			commandAccepted = 1;
		}
		else
		{
			USART3_printf("Invalid speed rank!\r\n");
		}
	}
	else
	{
		USART3_printf("Unknown command!\r\n");
	}

	if(commandAccepted)
	{
		RefreshCommandWatchdog();
	}
}

static uint8_t TakeTaskFlag(volatile uint8_t *flag)
{
	uint8_t ready;

	__disable_irq();
	ready = *flag;
	*flag = 0;
	__enable_irq();

	return ready;
}

static uint16_t TakeElapsedMs(volatile uint16_t *elapsedMs)
{
	uint16_t elapsed;

	__disable_irq();
	elapsed = *elapsedMs;
	*elapsedMs = 0;
	__enable_irq();

	return elapsed;
}

static void ServiceMpuTask(void)
{
	uint16_t elapsedMs = TakeElapsedMs(&MpuTaskElapsedMs);

	if(elapsedMs > 0)
	{
		MPU6050_Get_AngleDt(&MM, (float)elapsedMs * 0.001f);
		New_Pitch = KalmanFilter_Update(&Kal_Pitch,MM.pitch);
		New_Roll = KalmanFilter_Update(&Kal_Roll,MM.roll);
		New_Yaw = KalmanFilter_Update(&Kal_Yaw,MM.yaw);
	}
}

static void ServiceStraightTask(void)
{
	if(TakeTaskFlag(&StraightTaskReady) && is_straight)
	{
		keep_straight();
	}
}

int main ()
{
	KalmanFilter_Init(&Kal_Yaw,0.5,0.1,1,100);   // q=0.5: 稳态增益~83%,快速跟踪yaw变化(原0.01太慢仅吸收9%)
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
	SetServoRotation(90.0);
	
	Motor_Init();
	SetStandbyMode();
	RefreshCommandWatchdog();
	char commandBuffer[128];
	
	//校验MPU6050是否成功读到数据。
	USART3_printf("Everything is ready!\r\n");
	while(1)
	{
		//mpu_dmp_get_data(&MM.pitch,&MM.roll,&MM.yaw);
		//读取标志位就绪。
		if(USART3_ReadText(commandBuffer, sizeof(commandBuffer)) == 1)
		{
			HandleTextCommand(commandBuffer);
		}

		ServiceMpuTask();
		ServiceStraightTask();
		UpdateControlTask();

		
		//USART3_printf("%d,%d,%d,%d,%d,%d\r\n",MD.xAcc,MD.yAcc,MD.zAcc,MD.xGyro,MD.yGyro,MD.zGyro);
		
		//atan2:可以计算-180deg到180deg,第一个形参是分子。
		//USART3_printf("%f,%f,%f,%f,%f,%f,%f\r\n",EA.MPU6050_Yaw,EA.MPU6050_Roll,EA.MPU6050_Pitch,MD.zAcc*G*16/(0X7FFF),atan2(MD.xAcc,MD.yAcc)/PI*180,atan2(MD.yAcc,MD.zAcc)/PI*180,atan2(MD.xAcc,MD.zAcc)/PI*180);
		 //USART3_printf("%.3f,%.3f,%.3f\r\n", MM.roll, MM.pitch, MM.yaw);
		 //USART3_printf("%f,%f,%f,%d,%f,%f\r\n",rSpeed.Speed,lSpeed.Speed,aveSpeed,SpeedRank,rSpeed_PID.Out,lSpeed_PID.Out);
		 if(TakeTaskFlag(&TelemetryReady))
		 {
			 Odometry_t snapshot;
			 Odometry_GetSnapshot(&snapshot);
			 USART3_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",MM.GyroX/16.4f, New_Yaw, Angle, aveSpeed, snapshot.x, snapshot.y, headingPID.Kp, headingPID.Ki, headingPID.Kd);
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
		if(MpuTaskElapsedMs <= 995U)
		{
			MpuTaskElapsedMs += 5U;
		}
		
		//这里做一个读取丢包检测。如果一个数据超过8次没有任何变化那么认为MPU6050丢包,直接重新读取。
		
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
			StraightTaskReady = 1;
		}

		aveSpeed = (rSpeed.Speed + lSpeed.Speed)*0.5f;
	}	

	if(EXCOUNT(TelemetryCnt,100) == 1)
	{
		TelemetryReady = 1;
	}
}
