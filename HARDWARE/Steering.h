#ifndef _STEERING_H_
#define _STEERING_H_

#include "stm32f10x.h"

// ========== 舵机物理参数 ==========
#define SERVO_CENTER_DEG     90.0f   // 舵机中位角度
#define SERVO_MIN_DEG         0.0f   // 舵机最小角度
#define SERVO_MAX_DEG       180.0f   // 舵机最大角度

// ========== 开环转弯参数 (DT_TUR 专用) ==========
#define TURN_OPENLOOP_GAIN    2.0f   // °/dps: rate_dps → 舵机偏离中位的角度增益

// ========== 闭环转弯参数 (自动驾驶专用) ==========
#define YAW_RATE_MAX_DPS     90.0f   // 最大角速度 (°/s)
#define YAW_RATE_LUT_SIZE       19   // LUT 表项数 (servo 0°~180°, 每10°一个点)

// ========== 基础舵机控制 ==========
void Steering_SetAngle(float angle_deg);
float Steering_GetAngle(void);
void Steering_Center(void);

// ========== 开环转弯 (DT_TUR 专用, rate→servo 线性映射, 无陀螺仪反馈) ==========
void Steering_SetOpenLoopTurnRate(float rate_dps);
void Steering_StopOpenLoopTurn(void);
uint8_t Steering_IsOpenLoopTurnActive(void);

// ========== 闭环转弯 (自动驾驶专用, LUT前馈 + GyroZ PI反馈) ==========
void Steering_StartYawRateTurn(float target_rate_dps);
void Steering_UpdateYawRateControl(void);   // 每周期调用 (由 UpdateControlTask 调度)
void Steering_CancelYawRate(void);
uint8_t Steering_IsYawRateActive(void);

// ========== 定角转弯辅助 (UpdateYawTurn 调用) ==========
void Steering_SetYawTurnCorrection(float yaw_error_deg, int8_t direction);

// ========== LUT 查表 (公开以支持外部标定) ==========
float Steering_ServoToYawRate(float servo_angle_deg);
float Steering_YawRateToServo(float rate_dps);

#endif /* _STEERING_H_ */
