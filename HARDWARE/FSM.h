#ifndef _FSM_H_
#define _FSM_H_

#include "stm32f10x.h"
#include "Motors.h"

// ========== 超时与自动参数宏 ==========
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

// ========== 运行状态 (物理层) ==========
typedef enum RunState
{
    STANDBY = 0,
    PARKING,
    HITTED
} RS;

// ========== 控制模式 (任务层) ==========
typedef enum ControlMode
{
    CTRL_IDLE = 0,
    CTRL_MANUAL,
    CTRL_STRAIGHT,
    CTRL_DISTANCE,
    CTRL_TURN_YAW,
    CTRL_AUTO_ROUTE
} ControlMode_t;

// ========== 自动路线步进 (子任务层) ==========
typedef enum AutoStep
{
    AUTO_IDLE = 0,
    AUTO_FORWARD1,
    AUTO_TURN1,
    AUTO_FORWARD2
} AutoStep_t;

// ========== FSM 拥有的全局状态变量 ==========
extern RS rS;
extern uint8_t is_Pause;

// ========== 与 main.c 共享的全局变量 (由 main.c 定义) ==========
extern int8_t is_up;            // 方向: 1=前进, -1=后退
extern int8_t is_turn;          // 转弯标志
extern int8_t is_straight;      // 直行保持标志
extern float Angle;             // 舵机角度 (0~180°)
extern float New_Yaw;           // 卡尔曼滤波后的航向角
extern float Org_Yaw;           // 目标航向角 (直行保持锁定值)
extern volatile uint32_t ControlTicks;  // SysTick 计数器 (1ms/tick)
extern volatile uint8_t TelemetryReady; // 遥测输出标志

// ========== 工具函数 ==========
uint8_t ParseCommandValue(const char *s, float *value, uint8_t scaledHundredths);
float ClampFloat(float value, float minValue, float maxValue);
float AbsFloat(float value);
float NormalizeYaw(float yaw);
float GetYawError(float target, float current);

// ========== 模式切换 ==========
void RefreshCommandWatchdog(void);
void CenterSteering(void);
void HardStopMotion(void);
void SetStandbyMode(void);
void SetManualMode(void);
void SetManualModeIfIdle(void);
void EnsureAutoSpeed(void);
void PrepareStraightHold(void);

// ========== 自动驾驶 ==========
uint8_t PrepareDistanceDrive(float distanceCm);
void StartDistanceDrive(float distanceCm);
uint8_t PrepareYawTurn(float relativeYawDeg);
void StartYawTurn(float relativeYawDeg);
void StartAutoRoute(void);
uint8_t UpdateDistanceDrive(void);
uint8_t UpdateYawTurn(void);
void UpdateControlTask(void);

// ========== 速度与方向 ==========
void SpeedAcc(void);
void SpeedSlowDown(void);
void ExDirect(uint8_t Rot);
void SetSpeedRank(int8_t level);

// ========== 命令处理 (唯一入口) ==========
void HandleTextCommand(char *pBuffer);

// ========== 航向保持 (由 main.c 实现, FSM.c 调用) ==========
void Set_Straight(void);

#endif /* _FSM_H_ */
