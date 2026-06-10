#include "stm32f10x.h"
#include "CarControl.h"
#include "CarProtocol.h"
#include "Motors.h"
#include "PWMO.h"
#include "USART.h"
#include "LED.h"

#include <math.h>

int8_t is_up = 1;
uint8_t is_Pause = 1;
int8_t is_turn = 0;
int8_t is_straight = 0;
RS rS = STANDBY;

float Angle = 0.0f;
BMI270 MM;

float New_Yaw = 0.0f;
float New_Roll = 0.0f;
float New_Pitch = 0.0f;
float Org_Yaw = 0.0f;

KalmanFilter Kal_Yaw;
KalmanFilter Kal_Roll;
KalmanFilter Kal_Pitch;

volatile uint8_t TelemetryReady = 0;
volatile uint32_t ControlTicks = 0;

float TargetYaw = 0.0f;
uint8_t AutoSpeedLevel = AUTO_DEFAULT_SPEED;
uint8_t SpeedLimitLevel = AUTO_MAX_SPEED;
float SteerMinAngle = 0.0f;
float SteerMaxAngle = 180.0f;
float YawReportOffset = 0.0f;

typedef enum ControlMode
{
	CTRL_IDLE = 0,
	CTRL_MANUAL,
	CTRL_STRAIGHT,
	CTRL_DISTANCE,
	CTRL_TURN_YAW,
	CTRL_ARC,
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

static float AbsFloat(float value)
{
	return value < 0.0f ? -value : value;
}

float ClampFloat(float value, float minValue, float maxValue)
{
	if(value < minValue) return minValue;
	if(value > maxValue) return maxValue;
	return value;
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

float GetReportedYaw(void)
{
	return NormalizeYaw(New_Yaw - YawReportOffset);
}

const char *ControlModeName(void)
{
	switch(ControlMode)
	{
	case CTRL_IDLE: return "IDLE";
	case CTRL_MANUAL: return "MANUAL";
	case CTRL_STRAIGHT: return "STRAIGHT";
	case CTRL_DISTANCE: return "DISTANCE";
	case CTRL_TURN_YAW: return "TURN";
	case CTRL_ARC: return "ARC";
	case CTRL_AUTO_ROUTE: return "AUTO";
	default: return "UNKNOWN";
	}
}

const char *RunStateName(void)
{
	switch(rS)
	{
	case STANDBY: return "STANDBY";
	case PARKING: return "PARKING";
	case HITTED: return "HITTED";
	default: return "UNKNOWN";
	}
}

void RefreshCommandWatchdog(void)
{
	LastCommandTick = ControlTicks;
}

void SetSteeringAngle(float angle)
{
	Angle = ClampFloat(angle, SteerMinAngle, SteerMaxAngle);
	SetServoRotation(Angle);
}

static void CenterSteering(void)
{
	SetSteeringAngle(90.0f);
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

void PrintTelemetry(void)
{
	Odometry_t snapshot;

	Odometry_GetSnapshot(&snapshot);
	USART3_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
	              MM.GyroX/16.4f, GetReportedYaw(), Angle, aveSpeed, snapshot.x, snapshot.y,
	              headingPID.Kp, headingPID.Ki, headingPID.Kd);
}

void SetStandbyMode(void)
{
	HardStopMotion();
	rS = STANDBY;
	SetLEDs(GPIO_Pin_14);
}

void SetManualMode(void)
{
	if(IsAutoMotionMode())
	{
		AutoStep = AUTO_IDLE;
	}
	ControlMode = CTRL_MANUAL;
	is_straight = 0;
	is_turn = 0;
	headingPID.CrossTrackEnable = 0;
}

void SetManualModeIfIdle(void)
{
	if(ControlMode == CTRL_IDLE)
	{
		SetManualMode();
	}
}

uint8_t IsAutoMotionMode(void)
{
	return (ControlMode == CTRL_AUTO_ROUTE ||
	        ControlMode == CTRL_DISTANCE ||
	        ControlMode == CTRL_TURN_YAW ||
	        ControlMode == CTRL_ARC);
}

static void EnsureAutoSpeed(void)
{
	if(AutoSpeedLevel > SpeedLimitLevel)
	{
		AutoSpeedLevel = SpeedLimitLevel;
	}
	if(SpeedRank == 0)
	{
		SetSpeedRank(AutoSpeedLevel);
	}
}

void PrepareStraightHold(void)
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

uint8_t StartDistanceDrive(float distanceCm)
{
	if(PrepareDistanceDrive(distanceCm))
	{
		ControlMode = CTRL_DISTANCE;
		AutoStep = AUTO_IDLE;
		if(!CarProtocol_IsQuiet())
		{
			USART3_printf("Distance drive %.1f cm\r\n", TargetDistanceCm);
		}
		return 1;
	}

	if(!CarProtocol_IsQuiet())
	{
		USART3_printf("Invalid distance target!\r\n");
	}
	return 0;
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

uint8_t StartYawTurn(float relativeYawDeg)
{
	if(PrepareYawTurn(relativeYawDeg))
	{
		ControlMode = CTRL_TURN_YAW;
		AutoStep = AUTO_IDLE;
		if(!CarProtocol_IsQuiet())
		{
			USART3_printf("Yaw turn %.1f deg\r\n", relativeYawDeg);
		}
		return 1;
	}

	if(!CarProtocol_IsQuiet())
	{
		USART3_printf("Invalid yaw target!\r\n");
	}
	return 0;
}

uint8_t StartArcDrive(float distanceCm, float steerDeg)
{
	if(AbsFloat(distanceCm) < 1.0f)
	{
		if(!CarProtocol_IsQuiet())
		{
			USART3_printf("Invalid arc distance!\r\n");
		}
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

	is_straight = 0;
	is_turn = 0;
	headingPID.CrossTrackEnable = 0;
	SetSteeringAngle(steerDeg);
	Odometry_Reset();
	ActionStartTick = ControlTicks;
	EnsureAutoSpeed();
	ControlMode = CTRL_ARC;
	AutoStep = AUTO_IDLE;
	if(!CarProtocol_IsQuiet())
	{
		USART3_printf("Arc drive %.1f cm steer %.1f deg\r\n", TargetDistanceCm, Angle);
	}
	return 1;
}

uint8_t StartAutoRoute(void)
{
	rS = PARKING;
	SetLEDs(GPIO_Pin_12);
	AutoSpeedLevel = AUTO_DEFAULT_SPEED;
	ControlMode = CTRL_AUTO_ROUTE;
	AutoStep = AUTO_FORWARD1;
	if(PrepareDistanceDrive(AUTO_FORWARD1_CM))
	{
		if(!CarProtocol_IsQuiet())
		{
			USART3_printf("Auto route start\r\n");
		}
		return 1;
	}

	SetStandbyMode();
	if(!CarProtocol_IsQuiet())
	{
		USART3_printf("Auto route failed\r\n");
	}
	return 0;
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

	SetSteeringAngle(90.0f + correction);
	EnsureAutoSpeed();
	return 0;
}

void UpdateControlTask(void)
{
	if((ControlMode == CTRL_MANUAL || ControlMode == CTRL_STRAIGHT) &&
	   SpeedRank != 0 &&
	   (uint32_t)(ControlTicks - LastCommandTick) > REMOTE_TIMEOUT_MS)
	{
		SetStandbyMode();
		USART3_printf("Remote timeout stop!\r\n");
		return;
	}

	if((ControlMode == CTRL_DISTANCE || ControlMode == CTRL_ARC ||
	   (ControlMode == CTRL_AUTO_ROUTE && (AutoStep == AUTO_FORWARD1 || AutoStep == AUTO_FORWARD2))) &&
	   (uint32_t)(ControlTicks - ActionStartTick) > DISTANCE_TIMEOUT_MS)
	{
		SetStandbyMode();
		if(!CarProtocol_HasActiveMotion())
		{
			USART3_printf("Distance timeout stop!\r\n");
		}
		CarProtocol_FinishActiveMotionErr("TIMEOUT");
		return;
	}

	if((ControlMode == CTRL_TURN_YAW ||
	   (ControlMode == CTRL_AUTO_ROUTE && AutoStep == AUTO_TURN1)) &&
	   (uint32_t)(ControlTicks - ActionStartTick) > TURN_TIMEOUT_MS)
	{
		SetStandbyMode();
		if(!CarProtocol_HasActiveMotion())
		{
			USART3_printf("Turn timeout stop!\r\n");
		}
		CarProtocol_FinishActiveMotionErr("TIMEOUT");
		return;
	}

	if(ControlMode == CTRL_DISTANCE)
	{
		if(UpdateDistanceDrive())
		{
			SetStandbyMode();
			if(!CarProtocol_HasActiveMotion())
			{
				USART3_printf("Distance done\r\n");
			}
			CarProtocol_FinishActiveMotionOk("");
		}
	}
	else if(ControlMode == CTRL_ARC)
	{
		if(UpdateDistanceDrive())
		{
			SetStandbyMode();
			if(!CarProtocol_HasActiveMotion())
			{
				USART3_printf("Arc done\r\n");
			}
			CarProtocol_FinishActiveMotionOk("");
		}
	}
	else if(ControlMode == CTRL_TURN_YAW)
	{
		if(UpdateYawTurn())
		{
			SetStandbyMode();
			if(!CarProtocol_HasActiveMotion())
			{
				USART3_printf("Yaw turn done\r\n");
			}
			CarProtocol_FinishActiveMotionOk("");
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
					if(!CarProtocol_HasActiveMotion())
					{
						USART3_printf("Auto turn failed\r\n");
					}
					CarProtocol_FinishActiveMotionErr("AUTO_TURN_FAIL");
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
					if(!CarProtocol_HasActiveMotion())
					{
						USART3_printf("Auto forward failed\r\n");
					}
					CarProtocol_FinishActiveMotionErr("AUTO_FORWARD_FAIL");
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
				if(!CarProtocol_HasActiveMotion())
				{
					USART3_printf("Auto route done\r\n");
				}
				CarProtocol_FinishActiveMotionOk("");
			}
		}
	}
}

void SpeedAcc(void)
{
	int16_t rank = ABS(SpeedRank);

	if(rank < 720)
	{
		rank += SPEEDSTEP;
		if(rank > 720) rank = 720;
		SpeedRank = ABSTRACT(is_up) * rank;
	}
}

void SpeedSlowDown(void)
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
	Org_Yaw = New_Yaw;
	HeadingPID_Reset(&headingPID);
	Odometry_Reset();
	if(is_straight)
	{
		headingPID.CrossTrackEnable = 1;
	}
	is_Switch = 1;
}

void SetSpeedRank(int8_t level)
{
	if(level < 0)
	{
		level = 0;
	}
	if((uint8_t)level > SpeedLimitLevel)
	{
		level = (int8_t)SpeedLimitLevel;
	}
	if(level < 7)
	{
		SpeedRank = is_up * level * SPEEDSTEP;
	}
}

void SoftReset(void)
{
	__disable_irq();
	for(int i = 0; i < 10000; i++)
	{
	}
	NVIC_SystemReset();
}
