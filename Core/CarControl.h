#ifndef _CAR_CONTROL_H_
#define _CAR_CONTROL_H_

#include "MPU6050.h"
#include "filter.h"

#include <stdint.h>

#define FIRMWARE_VERSION        "SS928-CTRL-2.0"

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
#define AUTO_MAX_SPEED          6U

typedef enum RunState
{
	STANDBY = 0,
	PARKING,
	HITTED
} RS;

extern int8_t is_up;
extern uint8_t is_Pause;
extern int8_t is_turn;
extern int8_t is_straight;
extern RS rS;

extern float Angle;
extern MPU6050 MM;
extern float New_Yaw;
extern float New_Roll;
extern float New_Pitch;
extern float Org_Yaw;

extern KalmanFilter Kal_Yaw;
extern KalmanFilter Kal_Roll;
extern KalmanFilter Kal_Pitch;

extern volatile uint8_t TelemetryReady;
extern volatile uint32_t ControlTicks;

extern float TargetYaw;
extern uint8_t AutoSpeedLevel;
extern uint8_t SpeedLimitLevel;
extern float SteerMinAngle;
extern float SteerMaxAngle;
extern float YawReportOffset;

float ClampFloat(float value, float minValue, float maxValue);
float GetReportedYaw(void);
const char *ControlModeName(void);
const char *RunStateName(void);

void RefreshCommandWatchdog(void);
void SetSteeringAngle(float angle);
void SetStandbyMode(void);
void SetManualMode(void);
void SetManualModeIfIdle(void);
uint8_t IsAutoMotionMode(void);

void PrepareStraightHold(void);
uint8_t StartDistanceDrive(float distanceCm);
uint8_t StartYawTurn(float relativeYawDeg);
uint8_t StartArcDrive(float distanceCm, float steerDeg);
uint8_t StartAutoRoute(void);
void UpdateControlTask(void);

void PrintTelemetry(void);

void SpeedAcc(void);
void SpeedSlowDown(void);
void ExDirect(uint8_t Rot);
void SetSpeedRank(int8_t level);
void SoftReset(void);
void Set_Straight(void);
void keep_straight(void);

#endif
