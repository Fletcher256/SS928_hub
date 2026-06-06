#include "Steering.h"
#include "PWMO.h"
#include "USART.h"
#include "MPU6050.h"
#include <math.h>

// ========== 外部变量 (定义在 main.c) ==========
extern MPU6050 MM;
extern float Angle;     // 舵机角度全局变量, 供遥测输出使用

// ========== LUT 标定表: 舵机角度 → 稳态角速度 ==========
// 格式: servo 0°~180°, 每10°一个采样点, 共计19个点
// idx = servo/10, 线性插值
// 正值=左转, 负值=右转
// TODO: 需在平整地面上实测标定, 当前为占位值
static const float LUT_YawRate[YAW_RATE_LUT_SIZE] = {
    -90.0f,  //   0° 极右
    -85.0f,  //  10°
    -75.0f,  //  20°
    -60.0f,  //  30°
    -45.0f,  //  40°
    -25.0f,  //  50°
    -10.0f,  //  60°
     -3.0f,  //  70°
      0.0f,  //  80° 近中位死区
      0.0f,  //  90° 中位
      0.0f,  // 100° 近中位死区
      3.0f,  // 110°
     10.0f,  // 120°
     25.0f,  // 130°
     45.0f,  // 140°
     60.0f,  // 150°
     75.0f,  // 160°
     85.0f,  // 170°
     90.0f   // 180° 极左
};

// ========== 内部状态 ==========
static float CurrentAngle = SERVO_CENTER_DEG;  // 当前舵机角度

// ---- 开环转弯状态 ----
static float OpenLoopRate = 0.0f;  // 0=禁用

// ---- 闭环转弯状态 ----
static float YawRateTarget = 0.0f;     // 目标角速度 (dps), 0=禁用
static float YawRateIntegral = 0.0f;   // PI 积分项
static float YawRateLastError = 0.0f;  // 上一次误差

// ---- 闭环 PI 参数 (可在线标定) ----
#define YR_KP         0.5f    // 比例增益
#define YR_KI         0.05f   // 积分增益
#define YR_MAX_I     20.0f    // 积分限幅 (度)
#define YR_MAX_OUT   30.0f    // PI 输出限幅 (舵机偏离LUT基准的最大角度)

// ========== 基础舵机控制 ==========

void Steering_SetAngle(float angle_deg)
{
    if(angle_deg < SERVO_MIN_DEG) angle_deg = SERVO_MIN_DEG;
    if(angle_deg > SERVO_MAX_DEG) angle_deg = SERVO_MAX_DEG;
    CurrentAngle = angle_deg;
    Angle = angle_deg;  // 同步全局变量, 供遥测输出
    SetServoRotation(angle_deg);
}

float Steering_GetAngle(void)
{
    return CurrentAngle;
}

void Steering_Center(void)
{
    Steering_SetAngle(SERVO_CENTER_DEG);
}

// ========== LUT 查表 ==========

float Steering_ServoToYawRate(float servo_angle_deg)
{
    float idx_f;
    int idx;
    float frac;

    if(servo_angle_deg < 0.0f) servo_angle_deg = 0.0f;
    if(servo_angle_deg > 180.0f) servo_angle_deg = 180.0f;

    idx_f = servo_angle_deg / 10.0f;
    idx = (int)idx_f;
    if(idx >= YAW_RATE_LUT_SIZE - 1)
    {
        return LUT_YawRate[YAW_RATE_LUT_SIZE - 1];
    }

    frac = idx_f - (float)idx;
    return LUT_YawRate[idx] + (LUT_YawRate[idx + 1] - LUT_YawRate[idx]) * frac;
}

float Steering_YawRateToServo(float rate_dps)
{
    int i;
    float absRate;

    absRate = (rate_dps < 0.0f) ? -rate_dps : rate_dps;

    // 死区: |rate| < 3 dps → 回中
    if(absRate < 3.0f)
    {
        return SERVO_CENTER_DEG;
    }

    if(rate_dps > 0.0f)
    {
        // 左转 → 查找 >90° 区间
        for(i = YAW_RATE_LUT_SIZE - 1; i > 0; i--)
        {
            if(LUT_YawRate[i] <= rate_dps && LUT_YawRate[i-1] < LUT_YawRate[i])
            {
                // 在 LUT[i-1] ~ LUT[i] 之间插值
                float t = (rate_dps - LUT_YawRate[i-1]) / (LUT_YawRate[i] - LUT_YawRate[i-1]);
                return (float)(i - 1) * 10.0f + t * 10.0f;
            }
        }
        return 180.0f;
    }
    else
    {
        // 右转 → 查找 <90° 区间
        for(i = 0; i < YAW_RATE_LUT_SIZE - 1; i++)
        {
            if(LUT_YawRate[i] >= rate_dps && LUT_YawRate[i+1] > LUT_YawRate[i])
            {
                float t = (rate_dps - LUT_YawRate[i]) / (LUT_YawRate[i+1] - LUT_YawRate[i]);
                return (float)i * 10.0f + t * 10.0f;
            }
        }
        return 0.0f;
    }
}

// ========== 开环转弯 (DT_TUR 专用) ==========

void Steering_SetOpenLoopTurnRate(float rate_dps)
{
    float offset;
    float servo;

    if(fabs(rate_dps) < 0.1f)
    {
        Steering_StopOpenLoopTurn();
        return;
    }

    if(rate_dps > YAW_RATE_MAX_DPS) rate_dps = YAW_RATE_MAX_DPS;
    if(rate_dps < -YAW_RATE_MAX_DPS) rate_dps = -YAW_RATE_MAX_DPS;

    OpenLoopRate = rate_dps;
    offset = rate_dps * TURN_OPENLOOP_GAIN;
    servo = SERVO_CENTER_DEG + offset;

    Steering_SetAngle(servo);

    USART3_printf("Open-loop turn: %.1f dps -> servo %.0f deg\r\n",
                  (double)rate_dps, (double)servo);
}

void Steering_StopOpenLoopTurn(void)
{
    OpenLoopRate = 0.0f;
}

uint8_t Steering_IsOpenLoopTurnActive(void)
{
    return (fabs(OpenLoopRate) > 0.1f) ? 1 : 0;
}

// ========== 闭环转弯 (自动驾驶专用) ==========

void Steering_StartYawRateTurn(float target_rate_dps)
{
    if(fabs(target_rate_dps) < 0.1f)
    {
        Steering_CancelYawRate();
        return;
    }

    if(target_rate_dps > YAW_RATE_MAX_DPS) target_rate_dps = YAW_RATE_MAX_DPS;
    if(target_rate_dps < -YAW_RATE_MAX_DPS) target_rate_dps = -YAW_RATE_MAX_DPS;

    YawRateTarget = target_rate_dps;
    YawRateIntegral = 0.0f;
    YawRateLastError = 0.0f;

    // 初始舵机位置 = LUT 前馈值
    {
        float base_servo = Steering_YawRateToServo(target_rate_dps);
        Steering_SetAngle(base_servo);
    }

    USART3_printf("YawRate turn start: target %.1f dps\r\n", (double)target_rate_dps);
}

void Steering_UpdateYawRateControl(void)
{
    float gyro_z_dps;
    float error;
    float correction;
    float base_servo;
    float servo;

    if(fabs(YawRateTarget) < 0.1f)
    {
        return;
    }

    // 读取陀螺仪角速度 (°/s)
    gyro_z_dps = MM.GyroZ / 16.4f;

    // PI 反馈
    error = YawRateTarget - gyro_z_dps;

    // 积分累加
    YawRateIntegral += YR_KI * error;
    if(YawRateIntegral > YR_MAX_I)  YawRateIntegral = YR_MAX_I;
    if(YawRateIntegral < -YR_MAX_I) YawRateIntegral = -YR_MAX_I;

    // PI 输出
    correction = YR_KP * error + YawRateIntegral;
    if(correction > YR_MAX_OUT)  correction = YR_MAX_OUT;
    if(correction < -YR_MAX_OUT) correction = -YR_MAX_OUT;

    // LUT 前馈 + PI 反馈
    base_servo = Steering_YawRateToServo(YawRateTarget);
    servo = base_servo + correction;

    Steering_SetAngle(servo);

    YawRateLastError = error;
}

void Steering_CancelYawRate(void)
{
    YawRateTarget = 0.0f;
    YawRateIntegral = 0.0f;
    YawRateLastError = 0.0f;
}

uint8_t Steering_IsYawRateActive(void)
{
    return (fabs(YawRateTarget) > 0.1f) ? 1 : 0;
}

// ========== 定角转弯辅助 ==========

void Steering_SetYawTurnCorrection(float yaw_error_deg, int8_t direction)
{
    float correction;
    float servo;

    // yaw_error > 0 → 目标在左 → 需左转 → servo > 90
    correction = yaw_error_deg * 0.75f;  // TURN_SERVO_KP
    if(correction > 35.0f)  correction = 35.0f;   // TURN_SERVO_MAX_OFFSET
    if(correction < -35.0f) correction = -35.0f;

    // 后退时修正方向取反
    if(direction == -1)
    {
        correction = -correction;
    }

    servo = SERVO_CENTER_DEG + correction;
    Steering_SetAngle(servo);
}
